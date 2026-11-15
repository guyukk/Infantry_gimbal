#include <string.h>
#include "bsp_flash.c"
volatile struct {uint32_t status,checks,failure;} test_result;
static uint8_t storage[8192],source[512],dest[513];
static uint32_t erases,reads,writes,last_len,last_addr;
static int fail_erase,fail_read,fail_write;
int8_t OSPI_W25Qxx_Init(void){return 0;}
int8_t OSPI_W25Qxx_ReadBuffer(uint8_t *p,uint32_t a,uint32_t n){reads++;last_len=n;last_addr=a;if(fail_read)return -1;if(a+n<=sizeof(storage))memcpy(p,storage+a,n);else memset(p,0x5a,n);return 0;}
int8_t OSPI_W25Qxx_WriteBuffer(uint8_t *p,uint32_t a,uint32_t n){writes++;last_len=n;last_addr=a;if(fail_write)return -1;if(a+n<=sizeof(storage))memcpy(storage+a,p,n);return 0;}
int8_t OSPI_W25Qxx_SectorErase(uint32_t a){erases++;if(fail_erase)return -1;if(a+4096<=sizeof(storage))memset(storage+a,255,4096);return 0;}
static void ck(int y,uint32_t n){test_result.checks++;if(!y&&!test_result.failure)test_result.failure=n;}
static void reset(void){erases=reads=writes=0;fail_erase=fail_read=fail_write=0;}
void test_entry(void){
 reset();fail_erase=1;ck(flash_write_single_address(0,(uint32_t*)source,0)==0 && erases==0,201);
 reset();ck(flash_read(0,NULL,1)!=0 && !reads,202);
 reset();fail_erase=1;ck(flash_write_single_address(0,NULL,1)!=0 && !erases,203);
 reset();fail_erase=1;ck(flash_write_single_address(0,(uint32_t*)source,0x40000000)!=0 && !erases,204);
 reset();fail_erase=1;ck(flash_write_single_address(0x7ffffc,(uint32_t*)source,2)!=0 && !erases,205);
 reset();ck(flash_read(0x800000,(uint32_t*)dest,1)!=0 && !reads,206);
 reset();ck(flash_read(0,(uint32_t*)dest,0)==0 && !reads,207);
 for(unsigned i=0;i<sizeof(source);i++)source[i]=(uint8_t)i;
 reset();ck(flash_write_single_address(4092,(uint32_t*)source,2)==0 && erases==2 && writes==1,208);
 ck(!memcmp(storage+4092,source,8),209);
 reset();ck(flash_read(4092,(uint32_t*)(dest+1),2)==0 && !memcmp(dest+1,source,8),210);
 reset();ck(flash_write_single_address(0x7ffffc,(uint32_t*)source,1)==0 && erases==1 && last_len==4,211);
 reset();fail_erase=1;ck(flash_write_single_address(0,(uint32_t*)source,1)!=0 && !writes,212);
 reset();fail_write=1;ck(flash_write_single_address(0,(uint32_t*)source,1)!=0 && erases==1 && writes==1,213);
 reset();fail_read=1;ck(flash_read(0,(uint32_t*)dest,1)!=0 && reads==1,214);
 reset();ck(flash_read(0,(uint32_t*)(dest+1),100)==0 && !memcmp(dest+1,storage,400),215);
 test_result.status=test_result.failure?0xbad:0x600d;__asm("bkpt 0");
}

