#include "BMI088Middleware.h"
#include "main.h"

SPI_HandleTypeDef *BMI088_SPI;
static uint8_t bmi088_io_ok, bmi088_io_batch;
void BMI088_io_begin(void) { bmi088_io_ok=1U; bmi088_io_batch=1U; }
uint8_t BMI088_io_end(void) { bmi088_io_batch=0U; return bmi088_io_ok; }

void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_RESET);
}
void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_SET);
}

void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET);
}
void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);
}

uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data=0U;
    /* INS has higher priority than the motor task. Bound a failed batch and
       skip its remaining bytes so a stalled SPI cannot starve output guards. */
    if (bmi088_io_batch && !bmi088_io_ok) return 0U;
    if (HAL_SPI_TransmitReceive(BMI088_SPI, &txdata, &rx_data, 1, BMI088_SPI_TIMEOUT_MS) != HAL_OK)
        bmi088_io_ok=0U;
    return rx_data;
}
