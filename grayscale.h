#ifndef __GRAYSCALE_H__
#define __GRAYSCALE_H__

#include "ti_msp_dl_config.h"
#include <stdint.h>

/* 十二路灰度传感器通道数 */
/* Number of grayscale sensor channels */
#define GRAYSCALE_CHANNEL_COUNT  12

/**
 * @brief 读取单个灰度传感器通道的电平
 *        Read the level of a single grayscale sensor channel
 * @param channel 通道号 0~11  Channel number 0~11
 * @return 0=低电平, 1=高电平  0=low, 1=high
 */
uint8_t Grayscale_ReadChannel(uint8_t channel);

/**
 * @brief 读取全部12路灰度传感器, 结果存入16位变量
 *        Read all 12 grayscale channels, store result in a 16-bit variable
 * @return bit0~bit11 对应通道0~11的电平 (1=高, 0=低)
 *         bit0~bit11 correspond to channel 0~11 (1=high, 0=low)
 */
uint16_t Grayscale_ReadAll(void);

/**
 * @brief 通过串口打印全部12路灰度传感器的高低电平
 *        Print all 12 grayscale channel levels via UART
 */
void Grayscale_Print(void);

#endif /* __GRAYSCALE_H__ */
