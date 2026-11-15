"""Real health core for extracted-function tests; IRQ scheduling remains a stand-in."""
from pathlib import Path
import re
from project_paths import ROOT

def with_health(source, root=None):
    root=Path(root or ROOT)
    if not re.search(r'\bhealth_(touch|reject|fresh|gate_)',source):
        return source
    app='User/APP' if (root/'MDK-ARM/CtrlBoard-H7_IMU.uvprojx').exists() else 'application'
    header=(root/app/'health.h').read_text(encoding='utf-8')
    core=(root/app/'health.c').read_text(encoding='utf-8')
    core=re.sub(r'^#include "(health|main)\.h"\s*$', '', core,flags=re.M)
    # IRQ ordering is tested separately from these synchronous parser harnesses.
    macros='''#include <stdint.h>
#include <string.h>
#define __get_PRIMASK() 0U
#define __set_PRIMASK(value) ((void)(value))
#define __disable_irq() ((void)0)
'''
    for name in ['c_reset','g_reset','reset_all']:
        source=re.sub(r'(void\s+'+name+r'\s*\(void\)\s*\{)',r'\1\n health_reset();',source)
    return macros+header+'\n'+source+'\n'+core

