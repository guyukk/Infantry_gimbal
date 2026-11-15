"""Compile real board sources with ARMCC, execute bounded HAL fixtures in Unicorn."""
import argparse,hashlib,json,shutil,struct,subprocess,sys,re
from pathlib import Path
import xml.etree.ElementTree as ET
from project_paths import ROOT, ARM_BIN, DEPENDENCIES
sys.path.insert(0,str(DEPENDENCIES))
from elftools.elf.elffile import ELFFile
from unicorn import Uc,UC_ARCH_ARM,UC_MODE_THUMB,UC_MODE_MCLASS,UC_HOOK_CODE
from unicorn.arm_const import UC_ARM_REG_SP

def run(test,cache,out,mode='debug'):
    gimbal=(ROOT/'MDK-ARM/CtrlBoard-H7_IMU.uvprojx').exists()
    project=ROOT/('MDK-ARM/CtrlBoard-H7_IMU.uvprojx' if gimbal else 'MDK-ARM/standard_tpye_c.uvprojx')
    xml=ET.parse(project)
    controls=xml.find('./Targets/Target/TargetOption/TargetArmAds/Cads/VariousControls')
    includes=[(project.parent/x.replace('\\','/')).resolve() for x in controls.findtext('IncludePath').split(';') if x]
    cache.mkdir(parents=True,exist_ok=True);out.mkdir(parents=True,exist_ok=True)
    source=Path(__file__).with_name(test+'.c');obj=cache/(test+'.o');image=cache/(test+'.axf')
    compiler_source=source
    source_text=source.read_text(encoding='utf-8')
    if '/* B03_COMM_HOOKS */' in source_text:
        comm=(ROOT/'User/Communication/example/device/comm_app.c').read_text(encoding='utf-8')
        fragments=[]
        for name in ['gimbal_on_fire','gimbal_on_chassis','comm_apply_host_controls']:
            match=re.search(r'\b'+name+r'\([^;{}]*\)\s*\{',comm)
            assert match,name
            start=comm.rfind('\n',0,match.start())+1;end=match.end();depth=1
            while depth:
                depth+=(comm[end]=='{')-(comm[end]=='}');end+=1
            fragments.append(comm[start:end])
        compiler_source=cache/(test+'-generated.c')
        compiler_source.write_text(source_text.replace('/* B03_COMM_HOOKS */','\n'.join(fragments)),encoding='utf-8')
    if '/* B05_COMM_CALLBACKS */' in source_text:
        comm=(ROOT/'User/Communication/example/device/comm_app.c').read_text(encoding='utf-8')
        fragments=[]
        for name in ['gimbal_get_state','comm_publish_referee']:
            m=re.search(r'\b'+name+r'\([^;{}]*\)\s*\{',comm)
            assert m,name
            start=comm.rfind('\n',0,m.start())+1;end=m.end();depth=1
            while depth:
                depth+=(comm[end]=='{')-(comm[end]=='}');end+=1
            fragments.append(comm[start:end])
        if 'static bool gimbal_radians_to_udeg' in comm:
            m=re.search(r'static bool gimbal_radians_to_udeg[^{}]*\{',comm);end=m.end();depth=1
            while depth:
                depth+=(comm[end]=='{')-(comm[end]=='}');end+=1
            fragments.insert(0,comm[m.start():end])
        compiler_source=cache/(test+'-generated.c')
        compiler_source.write_text(source_text.replace('/* B05_COMM_CALLBACKS */','\n'.join(fragments)),encoding='utf-8')
    command=[str(ARM_BIN/'armcc.exe'),'--cpu','Cortex-M7' if gimbal else 'Cortex-M4','--fpu','SoftVFP','--thumb','--c99','--split_sections','-O0','-c',
             '-DUSE_HAL_DRIVER','-DSTM32H723xx' if gimbal else '-DSTM32F407xx','-DARM_MATH_CM7' if gimbal else '-DARM_MATH_CM4']
    if xml.findtext('./Targets/Target/TargetOption/TargetArmAds/Cads/uGnu') == '1':command.append('--gnu')
    command+=['-I'+str(x) for x in includes]+['-I'+str(ROOT),'-I'+str(Path(__file__).parent),str(compiler_source),'-o',str(obj)]
    if gimbal:command.insert(1,'-DROBOT_MODE='+('1' if mode=='release' else '0'))
    commands=[command,[str(ARM_BIN/'armlink.exe'),'--ro_base','0x10000','--rw_base','0x20000000','--entry','test_entry',str(obj),'-o',str(image)]]
    logs=[]
    for cmd in commands:
        result=subprocess.run(cmd,capture_output=True,timeout=90)
        logs.append(result.stdout.decode('gb18030','replace')+result.stderr.decode('gb18030','replace'))
        (out/(test+'-compiler.log')).write_text('\n'.join(logs),encoding='utf-8')
        (out/(test+'-commands.json')).write_text(json.dumps(commands,indent=2),encoding='utf-8')
        if result.returncode: raise RuntimeError(test+' compiler/linker failed; see '+str(out/(test+'-compiler.log')))
    uc=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS)
    uc.mem_map(0,0x100000);uc.mem_map(0x20000000,0x100000);uc.mem_map(0x40000000,0x100000)
    with image.open('rb') as f:
        elf=ELFFile(f)
        for segment in elf.iter_segments():
            if segment['p_type']=='PT_LOAD':uc.mem_write(segment['p_vaddr'],segment.data())
        for section in elf.iter_sections():
            if section['sh_flags'] & 2 and section['sh_type']=='SHT_PROGBITS':
                uc.mem_write(section['sh_addr'],section.data())
        symbols={s.name:s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}
        entry=elf['e_entry']
    uc.reg_write(UC_ARM_REG_SP,0x200FF000)
    def stop(machine,address,size,_):
        if machine.mem_read(address,2)==b'\x00\xbe':machine.emu_stop()
    uc.hook_add(UC_HOOK_CODE,stop)
    uc.emu_start(entry|1,0,timeout=30000000,count=12000000)
    values=struct.unpack('<3I',bytes(uc.mem_read(symbols['test_result'],12)))
    result=dict(status=hex(values[0]),checks=values[1],failure=values[2],passed=values[0]==0x600D and values[2]==0,
                scope='real firmware .c and headers; HAL/transport/RTOS stand-ins; no device or scheduler timing')
    if 'layout_values' in symbols:
        n=int.from_bytes(uc.mem_read(symbols['layout_count'],4),'little')
        result['layout_values']=list(struct.unpack('<'+'I'*n,bytes(uc.mem_read(symbols['layout_values'],4*n))))
    result['mode']=mode
    if 'test_robot_mode' in symbols:
        result['compiled_robot_mode']=int.from_bytes(uc.mem_read(symbols['test_robot_mode'],4),'little')
        assert result['compiled_robot_mode']==(1 if mode=='release' else 0), 'mode override did not take effect'
    if not result['passed']:
        result['failure_probes']={name:bytes(uc.mem_read(symbols[name],size)).hex() for name,size in [('sent_gimbal',8),('sent_fric',8),('sent_mit',8),('sources',260),('gates',64)] if name in symbols}
    inputs=[source,Path(__file__),project]+list((ROOT/('User' if gimbal else 'application')).rglob('*.c'))+list((ROOT/('User' if gimbal else 'application')).rglob('*.h'))
    inputs+=list((ROOT/('Core' if gimbal else 'UI')).rglob('*.c'))+list((ROOT/('Core' if gimbal else 'UI')).rglob('*.h'))
    result['input_sha256']={x.relative_to(ROOT).as_posix():hashlib.sha256(x.read_bytes()).hexdigest() for x in inputs}
    (out/(test+'-results.json')).write_text(json.dumps(result,indent=2),encoding='utf-8')
    shutil.copy2(image,out/image.name);shutil.copy2(source,out/source.name)
    if compiler_source!=source:shutil.copy2(compiler_source,out/compiler_source.name)
    print(json.dumps({k:v for k,v in result.items() if k!='input_sha256'}))
    if not result['passed']:raise RuntimeError(test+' failed '+str(result['failure']))
    return result

def main():
    p=argparse.ArgumentParser();p.add_argument('--cache',type=Path,required=True);p.add_argument('--results',type=Path,required=True);p.add_argument('--test',action='append');p.add_argument('--mode',choices=['debug','release'],default='debug')
    a=p.parse_args()
    g=(ROOT/'MDK-ARM/CtrlBoard-H7_IMU.uvprojx').exists()
    tests=a.test or (['test_b05_layout','test_b05_layout_usbfirst','test_b05_calibration','test_b05_persistence','test_b05_flash','test_b05_interface'] if g else ['test_b05_layout','test_b05_calibration','test_b05_ui'])
    for t in tests:run(t,a.cache.resolve(),a.results.resolve(),a.mode)
if __name__=='__main__':main()

