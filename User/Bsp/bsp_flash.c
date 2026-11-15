/* W25Q64 external Flash. Address is a byte offset; len is a 32-bit word count.
 * Nonzero writes erase every touched 4KB sector once and do not preserve bytes
 * outside the write range. Sector owners must read/modify/write when preserving them. */
#include "bsp_flash.h"
#include "w25q64.h"
#define W25Q_SECTOR 4096U
static uint8_t flash_range_valid(uint32_t address, const void *buf, uint32_t len)
{
    return address <= W25Qxx_FlashSize && (len==0U ||
           (buf!=0 && len <= (W25Qxx_FlashSize-address)/4U));
}
int8_t flash_init(void){return OSPI_W25Qxx_Init()==OSPI_W25Qxx_OK ? 0:-1;}
int8_t flash_read(uint32_t address,uint32_t *buf,uint32_t len)
{
    if(!flash_range_valid(address,buf,len)) return -1;
    if(len==0U) return 0;
    return OSPI_W25Qxx_ReadBuffer((uint8_t*)buf,address,len*4U)==OSPI_W25Qxx_OK ? 0:-1;
}
int8_t flash_write_single_address(uint32_t address,uint32_t *buf,uint32_t len)
{
    uint32_t first,last,s;
    if(!flash_range_valid(address,buf,len)) return -1;
    if(len==0U) return 0;
    first=address/W25Q_SECTOR;last=(address+len*4U-1U)/W25Q_SECTOR;
    for(s=first;s<=last;s++)
        if(OSPI_W25Qxx_SectorErase(s*W25Q_SECTOR)!=OSPI_W25Qxx_OK) return -1;
    return OSPI_W25Qxx_WriteBuffer((uint8_t*)buf,address,len*4U)==OSPI_W25Qxx_OK ? 0:-1;
}
void flash_erase_address(uint32_t address,uint16_t count)
{
    /* Legacy unused void API; active writers use checked flash_write_single_address. */
    uint32_t i;
    if(address%W25Q_SECTOR || address>W25Qxx_FlashSize ||
       count>(W25Qxx_FlashSize-address)/W25Q_SECTOR) return;
    for(i=0;i<count;i++) if(OSPI_W25Qxx_SectorErase(address+i*W25Q_SECTOR)!=OSPI_W25Qxx_OK) return;
}
