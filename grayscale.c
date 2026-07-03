#include "grayscale.h"
#include "usart.h"

/* ================================================================
 *  十二路灰度传感器引脚查找表
 *  12-way grayscale sensor pin lookup table
 *
 *  所有引脚位于 PORTB:
 *  CH0  -> PB0  (Grays_PIN_0)
 *  CH1  -> PB1  (Grays_PIN_1)
 *  CH2  -> PB2  (Grays_PIN_2)
 *  CH3  -> PB3  (Grays_PIN_3)
 *  CH4  -> PB4  (Grays_PIN_4)
 *  CH5  -> PB5  (Grays_PIN_5)
 *  CH6  -> PB6  (Grays_PIN_6)
 *  CH7  -> PB7  (Grays_PIN_7)
 *  CH8  -> PB8  (Grays_PIN_8)
 *  CH9  -> PB9  (Grays_PIN_9)
 *  CH10 -> PB10 (Grays_PIN_10)
 *  CH11 -> PB11 (Grays_PIN_11)
 * ================================================================ */

static const uint32_t gGrayscalePins[GRAYSCALE_CHANNEL_COUNT] = {
    Grays_PIN_0_PIN,   /* CH0  */
    Grays_PIN_1_PIN,   /* CH1  */
    Grays_PIN_2_PIN,   /* CH2  */
    Grays_PIN_3_PIN,   /* CH3  */
    Grays_PIN_4_PIN,   /* CH4  */
    Grays_PIN_5_PIN,   /* CH5  */
    Grays_PIN_6_PIN,   /* CH6  */
    Grays_PIN_7_PIN,   /* CH7  */
    Grays_PIN_8_PIN,   /* CH8  */
    Grays_PIN_9_PIN,   /* CH9  */
    Grays_PIN_10_PIN,  /* CH10 */
    Grays_PIN_11_PIN,  /* CH11 */
};

/**
 * @brief 读取单个灰度传感器通道的电平
 *        Read the level of a single grayscale sensor channel
 * @param channel 通道号 0~11
 * @return 0=低电平(黑/暗), 1=高电平(白/亮)
 *         0=low (dark), 1=high (bright)
 */
uint8_t Grayscale_ReadChannel(uint8_t channel)
{
    if (channel >= GRAYSCALE_CHANNEL_COUNT)
    {
        return 0;
    }

    /* DL_GPIO_readPins 返回该引脚所在位的值: 高电平返回非零, 低电平返回0
       DL_GPIO_readPins returns the pin bit value: non-zero = high, 0 = low */
    if (DL_GPIO_readPins(Grays_PORT, gGrayscalePins[channel]) &
        gGrayscalePins[channel])
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief 读取全部12路灰度传感器, 结果存入16位变量
 *        Read all 12 grayscale channels, return a 16-bit bitmap
 * @return bit0~bit11 对应通道0~11 (1=高电平, 0=低电平)
 *         bit0~bit11 correspond to channel 0~11 (1=high, 0=low)
 */
uint16_t Grayscale_ReadAll(void)
{
    uint16_t result = 0;
    uint8_t  i;

    for (i = 0; i < GRAYSCALE_CHANNEL_COUNT; i++)
    {
        if (DL_GPIO_readPins(Grays_PORT, gGrayscalePins[i]) &
            gGrayscalePins[i])
        {
            result |= (1 << i);
        }
    }

    return result;
}

/**
 * @brief 通过串口打印全部12路灰度传感器的高低电平
 *        Print all 12 grayscale channel levels via UART
 *
 * 输出格式: "Grayscale: CH00:1 CH01:0 CH02:1 ... CH11:0\r\n"
 * Output format: high=1, low=0
 */
void Grayscale_Print(void)
{
    uint8_t i;
    uint8_t level;
    char    buf[8];

    USART_SendString("Grayscale: ");

    for (i = 0; i < GRAYSCALE_CHANNEL_COUNT; i++)
    {
        level = Grayscale_ReadChannel(i);

        /* 组装 "CHxx:x" 字符串  Build "CHxx:x" string */
        buf[0] = 'C';
        buf[1] = 'H';
        buf[2] = '0' + (i / 10);
        buf[3] = '0' + (i % 10);
        buf[4] = ':';
        buf[5] = '0' + level;
        buf[6] = '\0';
        USART_SendString(buf);

        /* 通道间加空格, 行末加回车换行
           Space between channels, CR+LF at end of line */
        if (i < GRAYSCALE_CHANNEL_COUNT - 1)
        {
            USART_SendData(' ');
        }
    }

    USART_SendString("\r\n");
}
