#include "usart.h"
#include <stddef.h>


#define RE_0_BUFF_LEN_MAX	128

volatile uint8_t  recv0_buff[RE_0_BUFF_LEN_MAX] = {0};
volatile uint16_t recv0_length = 0;
volatile uint8_t  recv0_flag = 0;

void USART_Init(void)
{
	//清除串口中断标志
	//Clear the serial port interrupt flag
	NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
	//使能串口中断
	//Enable serial port interrupt
	NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
}

//串口发送一个字节
//The serial port sends a byte
void USART_SendData(unsigned char data)
{
	//当串口0忙的时候等待
	//Wait when serial port 0 is busy
	while( DL_UART_isBusy(UART_0_INST) == true );
	//发送
	//send
	DL_UART_Main_transmitData(UART_0_INST, data);
}

//串口发送字符串
//The serial port sends a string
void USART_SendString(const char *str)
{
    while (*str != '\0')
    {
        USART_SendData((unsigned char)*str++);
    }
}


//重定向_write_r函数 (ti-clang / newlib 底层可重入接口)
//Redirect _write_r function (ti-clang / newlib low-level reentrant interface)
//printf 等标准库函数最终调用 _write_r, 而非 _write
//printf and other stdio functions ultimately call _write_r, not _write
struct _reent;

int _write_r(struct _reent *r, int fd, const char *buf, size_t count)
{
    size_t i;
    (void)r;    /* 未使用 unused */
    for (i = 0; i < count; i++)
    {
        while (DL_UART_isBusy(UART_0_INST) == true);
        DL_UART_Main_transmitData(UART_0_INST, buf[i]);
    }
    return count;
}

//串口的中断服务函数
//Serial port interrupt service function
void UART_0_INST_IRQHandler(void)
{
	uint8_t receivedData = 0;
	
	//如果产生了串口中断
	//If a serial port interrupt occurs
	switch( DL_UART_getPendingInterrupt(UART_0_INST) )
	{
		case DL_UART_IIDX_RX://如果是接收中断	If it is a receive interrupt
			
			// 接收发送过来的数据保存	Receive and save the data sent
			receivedData = DL_UART_Main_receiveData(UART_0_INST);
		
			// 检查缓冲区是否已满	Check if the buffer is full
			if (recv0_length < RE_0_BUFF_LEN_MAX - 1)
			{
				recv0_buff[recv0_length++] = receivedData;
			}
			else
			{
				recv0_length = 0;
			}

			// 标记接收标志	Mark receiving flag
			recv0_flag = 1;
		
			break;
		
		default://其他的串口中断	Other serial port interrupts
			break;
	}
}
