#ifndef _VOFA_H
#define _VOFA_H

#include "usart.h"
#include <stdio.h>
#include "stdint.h"
#include <string.h>
/* 只写数组名，自动推元素个数 */
#define Vofa_JustFloat_AUTO(arr) Vofa_JustFloat((arr), (uint8_t)(sizeof(arr) / sizeof((arr)[0])))

/* 调试输出只走 USART10；不允许占用 USART1 板间协议链路 */
void Vofa_JustFloat(float *_data, uint8_t _num);
/* 发送/丢弃计数，便于判断调试输出是否被 BUSY 吞掉 */
void vofa_get_stats(uint32_t *sent, uint32_t *dropped);

#endif
