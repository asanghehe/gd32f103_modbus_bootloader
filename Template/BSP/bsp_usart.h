#ifndef __USART_H
#define	__USART_H


#include "gd32f10x.h"
#include <stdio.h>

/** 
  * 串口宏定义，不同的串口挂载的总线和IO不一样，移植时需要修改这几个宏
	* 1-修改总线时钟的宏，uart1挂载到apb2总线，其他uart挂载到apb1总线
	* 2-修改GPIO的宏
  */

#define COM3

#if defined(COM2)

#define COM_PORT	USART2

#define COM_PORT_T	GPIOB
#define COM_PORT_R	GPIOB
#define COM_PORT_PIN_TX	GPIO_PIN_10
#define COM_PORT_PIN_RX	GPIO_PIN_11

#define COM_PORT_IRQ_HANDLER		USART2_IRQHandler

#define COM_PORT_IRQ			USART2_IRQn

#define COM_PORT_RCU			RCU_USART2


//DMA config
#define DMA_DEVICE		DMA0
#define DMA_RCU			RCU_DMA0

#define USART_RX_DMA_CHANNEL     DMA_CH2
#define DMA_USART_RX_IRQ				DMA1_Channel2_IRQn
#define DMA_USART_RX_HANDLER 		DMA1_Channel2_IRQHandler

#define USART_TX_DMA_CHANNEL     DMA_CH4
#define DMA_USART_TX_IRQ				DMA1_Channel3_Channel4_IRQn
#define DMA_USART_TX_HANDLER 		DMA1_Channel3_4_IRQHandler

#elif defined(COM3)

#define COM_PORT	UART3

#define COM_PORT_T	GPIOC
#define COM_PORT_R	GPIOC
#define COM_PORT_PIN_TX	GPIO_PIN_10
#define COM_PORT_PIN_RX	GPIO_PIN_11

#define COM_PORT_IRQ_HANDLER		UART3_IRQHandler

#define COM_PORT_IRQ			UART3_IRQn

#define COM_PORT_RCU			RCU_UART3


//DMA config
#define DMA_DEVICE		DMA1
#define DMA_RCU			RCU_DMA1

#define USART_RX_DMA_CHANNEL     DMA_CH2
#define DMA_USART_RX_IRQ				DMA1_Channel2_IRQn
#define DMA_USART_RX_HANDLER 		DMA1_Channel2_IRQHandler

#define USART_TX_DMA_CHANNEL     DMA_CH4
#define DMA_USART_TX_IRQ				DMA1_Channel3_Channel4_IRQn
#define DMA_USART_TX_HANDLER 		DMA1_Channel3_4_IRQHandler

#elif defined(COM4)

#define COM_PORT	UART4

#define COM_PORT_T	GPIOC
#define COM_PORT_R	GPIOD
#define COM_PORT_PIN_TX	GPIO_PIN_12
#define COM_PORT_PIN_RX	GPIO_PIN_2

#define COM_PORT_IRQ_HANDLER		UART4_IRQHandler

#define COM_PORT_IRQ			UART4_IRQn

#define COM_PORT_RCU			RCU_UART4


//DMA config
#define DMA_DEVICE		DMA1
#define DMA_RCU			RCU_DMA1

#define USART_RX_DMA_CHANNEL     DMA_CH2
#define DMA_USART_RX_IRQ				DMA1_Channel2_IRQn
#define DMA_USART_RX_HANDLER 		DMA1_Channel2_IRQHandler

#define USART_TX_DMA_CHANNEL     DMA_CH4
#define DMA_USART_TX_IRQ				DMA1_Channel3_Channel4_IRQn
#define DMA_USART_TX_HANDLER 		DMA1_Channel3_4_IRQHandler

#endif

/* 串口/ DMA 的初始化与收发均在 freemodbus/port/rtt/portserial.c 中实现,
   本头文件只提供 COM_PORT / DMA 通道等引脚与通道映射宏。 */

#endif /* __USART_H */
