#include "delay.h"

volatile unsigned int delay_times = 0;

// 使用 SysTick 定时器实现的精确 us 延时
// Accurate us delay with tick timer
void delay_us(unsigned long __us) 
{
    uint32_t ticks;
    uint32_t told, tnow, tcnt = 38;

    // 计算所需时钟数 = 延时微秒数 * 每微秒的时钟数
	// Calculate the number of clocks required = delay microseconds * number of clocks per microsecond
    ticks = __us * (32000000 / 1000000);

    // 获取当前的 SysTick 值
	// Get the current SysTick value
    told = SysTick->VAL;

    while (1)
    {
        // 重复刷新获取当前的 SysTick 值
		// Repeatedly refresh to get the current SysTick value
        tnow = SysTick->VAL;

        if (tnow != told)
        {
            if (tnow < told)
                tcnt += told - tnow;
            else
                tcnt += SysTick->LOAD - tnow + told;

            told = tnow;

            // 如果达到所需的时钟数，退出循环
			// If the required number of clocks is reached, exit the loop
            if (tcnt >= ticks)
                break;
        }
    }
}
// 使用 SysTick 定时器实现的精确 ms 延时
// Accurate ms delay with tick timer
void delay_ms(unsigned long ms) 
{
	delay_us( ms * 1000 );
}

//void SysTick_Handler(void)
//{
//	if(delay_times != 0)
//	{
//		delay_times--;
//	}
//}
