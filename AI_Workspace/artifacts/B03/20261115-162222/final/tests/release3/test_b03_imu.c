#include <string.h>
#include "BMI088driver.c"
#include "BMI088Middleware.c"
volatile struct{uint32_t status,checks,failure;}test_result;
static uint32_t calls,fail_at;
static uint8_t script[24];
static uint32_t timeout_seen;
void HAL_GPIO_WritePin(GPIO_TypeDef*g,uint16_t pin,GPIO_PinState state){(void)g;(void)pin;(void)state;}
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef*h,const uint8_t*tx,uint8_t*rx,uint16_t n,uint32_t timeout){
 (void)h;(void)tx;(void)n;timeout_seen=timeout;*rx=script[calls];calls++;return calls==fail_at?HAL_TIMEOUT:HAL_OK;
}
void DWT_Delay(float t){(void)t;}
void HAL_Delay(uint32_t t){(void)t;}
float DWT_GetTimeline_s(void){return 0;}
static void check(uint8_t ok,uint32_t n){test_result.checks++;if(!ok){test_result.failure=n;__asm("bkpt 0");}}
static void good(void){memset(script,0,sizeof(script));script[2]=BMI088_ACC_CHIP_ID_VALUE;script[12]=BMI088_GYRO_CHIP_ID_VALUE;calls=0;fail_at=0;BMI088.AccelScale=1;}
void test_entry(void){
 uint32_t i;
 good();BMI088_Read(&BMI088);check(BMI088_sample_valid&&calls==24,1);check(timeout_seen==1U,2);
 good();script[2]=0xFF;BMI088_Read(&BMI088);check(!BMI088_sample_valid,3);
 good();script[12]=0xFF;BMI088_Read(&BMI088);check(!BMI088_sample_valid,4);
 for(i=1;i<=24;i++){good();fail_at=i;BMI088_Read(&BMI088);check(!BMI088_sample_valid&&calls==i,100+i);}
 good();BMI088_Read(&BMI088);check(BMI088_sample_valid,5);
 test_result.status=0x600D;__asm("bkpt 0");
}

