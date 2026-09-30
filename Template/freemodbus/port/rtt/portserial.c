/*
 * FreeModbus Libary: RT-Thread Port
 * Copyright (C) 2013 Armink <armink.ztl@gmail.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: portserial.c,v 1.60 2013/08/13 15:07:05 Armink $
 */

#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "bsp_usart.h"

#include "systick.h"

#include "gd32f10x_timer.h" 

#include "bsp_TimBase.h"
#include "bsp_led.h" 

/* ----------------------- Defines ------------------------------------------*/
/* DELAY_TIM 单次定时的长度(us)，与 porttimer.c 中 period = 299（1us 计数）对应 */
#define USART_TX_DELAY_TICK_US      300U

/* 485 发送方向最长允许停留时间(ms)。
   正常路径由 DMA 完成中断 + DELAY_TIM 中断把方向切回接收；
   该中断链任一环丢失，485 会停在发送使能方向导致彻底收不到数据。
   超过该时间仍未切回接收，则由主循环看门狗强制切回。*/
#define MB_485_TX_WATCHDOG_MS       100U

/* ----------------------- static functions ---------------------------------*/
static void prvvUARTTxDmaInit( void );

/* ----------------------- static variables ---------------------------------*/
/* 发送完成后切回接收方向前必须等够的 DELAY_TIM 定时次数，按波特率在 xMBPortSerialInit 中计算 */
static uint16_t ucTxWaitTicks = 2;
/* 当前已经等待的定时次数 */
static uint16_t ucTxWaitCnt = 0;

/* 485 方向软件标志：TRUE 表示当前处于发送使能方向 */
static volatile BOOL s_bRs485InTx = FALSE;
/* 进入发送使能方向时的系统节拍(ms)，供发送看门狗做超时判断 */
static volatile ULONG s_ulTxDirEnterMs = 0;

/* ----------------------- Start implementation -----------------------------*/
BOOL xMBPortSerialInit(UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits,
        eMBParity eParity, UCHAR ucStopBits)
{

	    /* enable USART clock */
    rcu_periph_clock_enable(COM_PORT_RCU);
    
		/* enable GPIO clock */
		rcu_periph_clock_enable(RCU_GPIOA);
		rcu_periph_clock_enable(RCU_GPIOB);
		rcu_periph_clock_enable(RCU_GPIOC);
		rcu_periph_clock_enable(RCU_GPIOD);
		
		/* connect port to USARTx_Tx */
		gpio_init(COM_PORT_T, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, COM_PORT_PIN_TX);
		/* connect port to USARTx_Rx */
		gpio_init(COM_PORT_R, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, COM_PORT_PIN_RX);

    /* USART configure */
    usart_deinit(COM_PORT);
    usart_baudrate_set(COM_PORT, ulBaudRate);
    usart_word_length_set(COM_PORT, USART_WL_8BIT);
    usart_stop_bit_set(COM_PORT, USART_STB_1BIT);
    usart_parity_config(COM_PORT, USART_PM_NONE);
    usart_hardware_flow_rts_config(COM_PORT, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(COM_PORT, USART_CTS_DISABLE);
	
    usart_receive_config(COM_PORT, USART_RECEIVE_ENABLE);
    usart_transmit_config(COM_PORT, USART_TRANSMIT_ENABLE);
    usart_enable(COM_PORT);
		
		
		
    usart_interrupt_disable(COM_PORT, USART_INT_TBE | USART_INT_RBNE);
		
		/* 接收中断优先级必须低于 DELAY_TIM(5)：DMA 发完后要先由 DELAY_TIM 完成
		   “切回 485 接收方向 + 清掉残留字节”，之后才允许 RBNE 抢占；否则
		   切换期间的残字节会被协议栈当成新帧首字节，吃掉下一次请求。 */
		nvic_irq_enable(COM_PORT_IRQ, 6, 1);
		NVIC_DisableIRQ(COM_PORT_IRQ);
		//nvic_irq_disable(COM_PORT_IRQ);

		/* 计算"一帧发完 -> 切回 485 接收方向"需要等待的时间：
		   DMA 传输完成中断是在最后一个字节刚写进 DR 时产生的，该字节还要等移位寄存器
		   空出来（最多 1 个字节时间）再完整移出（又 1 个字节时间），最坏共 2 个字节时间，
		   这里按 3 个字节时间（1 字节 = 10bit）留余量；等不够就松开发送使能会把最后一个
		   字节的 bit7 截断成 1，现象是响应帧 CRC 恒差 0x8000。 */
		{
				ULONG ulByteTime = 10000000UL / ulBaudRate;         /* 1 个字节(10bit)的微秒数 */
				ULONG ulNeedTime = ulByteTime * 3UL;                /* 3 个字节时间 */

				ucTxWaitTicks = (uint16_t)((ulNeedTime + USART_TX_DELAY_TICK_US - 1UL) / USART_TX_DELAY_TICK_US);
				if( ucTxWaitTicks < 2U ){
						ucTxWaitTicks = 2U;
				}
		}

		/* 发送改为 DMA，初始化发送 DMA 通道与中断 */
		prvvUARTTxDmaInit();

		return TRUE;
		}

void vMBPortSerialEnable(BOOL rxEnable, BOOL txEnable)
{
		//nvic_irq_disable(COM_PORT_IRQ);
		NVIC_DisableIRQ(COM_PORT_IRQ);
    
		/* 发送已交由 DMA 完成，TBE 中断始终关闭 */
		usart_interrupt_disable(COM_PORT, USART_INT_TBE);
		
		if( rxEnable ){
        /* 这里先不开 RBNE：此刻 485 仍处在发送方向，要等 DELAY_TIM 到点才切回接收，
           这段时间收到的都是自身回显/切换残字节，一旦开 RBNE 就会被协议栈当成
           新帧首字节、吃掉下一次请求。统一改由 DELAY_TIM_IRQHandler 在
           “切回接收方向 -> 清掉残留字节”之后再打开 RBNE。 */
        usart_interrupt_disable(COM_PORT, USART_INT_RBNE);
        NVIC_ClearPendingIRQ(COM_PORT_IRQ);
				
				//延迟开启接收，发送最后一个可能没有完成
				ucTxWaitCnt = 0;
				timer_interrupt_flag_clear(DELAY_TIM, TIMER_INT_FLAG_UP);
				timer_interrupt_enable(DELAY_TIM, TIMER_INT_UP);
				timer_enable(DELAY_TIM);
			
				LED_COM_OFF;
		}
    else{
        usart_interrupt_disable(COM_PORT, USART_INT_RBNE);

        /* 取消尚未完成的“切回收方向”延时，防止它在本帧发送途中把 485 切回接收 */
        timer_interrupt_disable(DELAY_TIM, TIMER_INT_UP);
        timer_disable(DELAY_TIM);
		}
		
		if( txEnable ){
				/* 切换 485 为发送方向，数据由 xMBPortSerialDmaPut() 启动 DMA 搬移 */
				LED_485_OFF;
		
				LED_COM_ON;
		
				/* 记录进入发送方向的时间，供发送看门狗做超时兜底 */
				s_ulTxDirEnterMs = systick_get_ms();
				s_bRs485InTx = TRUE;
		
				//使用DMA发送，不需要这些
				//启用中断会立即触发中断事件，开始传输
				//等于调用：pxMBFrameCBTransmitterEmpty
       			//usart_interrupt_enable(COM_PORT, USART_INT_TBE);
		}else{	
				//使用DMA发送，不需要这些
        		//usart_interrupt_disable(COM_PORT, USART_INT_TBE);
		}
		
		/* 收发全部关闭（如 eMBRTUStop）：确保 485 回到接收方向并复位发送标志，
		   避免发送看门狗把停止状态误判为需要兜底 */
		if( !rxEnable && !txEnable ){
				LED_485_ON;
				s_bRs485InTx = FALSE;
		}

		// Re-enable UART interrupt only if at least one direction is active
    if( rxEnable || txEnable )
				//nvic_irq_enable(COM_PORT_IRQ, 3, 1);
				NVIC_EnableIRQ(COM_PORT_IRQ);
}

void vMBPortClose(void)
{
		
}

BOOL xMBPortSerialPutByte(CHAR ucByte)
{

		usart_data_transmit(COM_PORT, (uint8_t) ucByte );
	
    return TRUE;
}

BOOL xMBPortSerialGetByte(CHAR * pucByte)
{
		* pucByte = usart_data_receive(COM_PORT);
    return TRUE;
}

int error_count = 0;
int sendcount = 0;


void DELAY_TIM_IRQHandler( void )
{
		if(SET == timer_interrupt_flag_get(DELAY_TIM, TIMER_INT_FLAG_UP)){
        /* clear channel 0 interrupt bit */
        timer_interrupt_flag_clear(DELAY_TIM, TIMER_INT_FLAG_UP);
        /* 同步清 NVIC pending，避免退出后再次误入本中断 */
        NVIC_ClearPendingIRQ(DELAY_TIM_IRQ);

				/* 必须按字节时间等够再切方向：DMA 完成中断时最后一个字节可能刚写进 DR，
				   最坏还要 2 个字节时间才完整移出，提前松开 485 发送使能会把它的
				   bit7 截断成 1（总线提前回空闲电平），响应帧 CRC 就会恒差 0x8000。 */
				if( ++ucTxWaitCnt < ucTxWaitTicks ){
						timer_enable(DELAY_TIM);
						return;
				}

				/* 1) 先把 485 切回接收方向 */
				LED_485_ON;
				s_bRs485InTx = FALSE;

				/* 2) 丢弃发送/收发切换期间残留在接收寄存器里的无效字节，
				      否则它会被协议栈当成新帧的起始字节吃掉下一次请求 */
				if(RESET != usart_flag_get(COM_PORT, USART_FLAG_RBNE)){
						(void)USART_DATA(COM_PORT);
				}
				usart_flag_clear(COM_PORT, USART_FLAG_ORERR);
				usart_interrupt_flag_clear(COM_PORT, USART_INT_FLAG_RBNE);
				NVIC_ClearPendingIRQ(COM_PORT_IRQ);

				/* 3) 残字节清干净后，最后才真正打开接收中断 */
				usart_interrupt_enable(COM_PORT, USART_INT_RBNE);

    }
}


void COM_PORT_IRQ_HANDLER(void)
{
	
    if(RESET != usart_interrupt_flag_get(COM_PORT, USART_INT_FLAG_RBNE)){
				usart_interrupt_flag_clear(COM_PORT, USART_INT_FLAG_RBNE);
        /* receive data */
        pxMBFrameCBByteReceived();
    }
	
	/** //使用DMA发送，不需要这些
    if(RESET != usart_interrupt_flag_get(COM_PORT, USART_INT_FLAG_TBE)){
				usart_interrupt_flag_clear(COM_PORT, USART_INT_FLAG_TC| USART_INT_FLAG_RBNE);
				
		//timer_counter_value_config(TIMER5, 0);
		//timer_enable(TIMER5);
			
        //transmit data
        pxMBFrameCBTransmitterEmpty();
			
		//sendcount = timer_counter_read(TIMER5);
		//if(sendcount > 100){
		//	error_count++;
		//}
		//timer_disable(TIMER5);
    }
 	*/
}


/**
 * @brief  发送 DMA 初始化
 *         使用 bsp_usart.h 中配置的 DMA 控制器、DMA 通道与 DMA 中断
 */
static void prvvUARTTxDmaInit( void )
{
	dma_parameter_struct dma_init_struct;

	/* enable DMA clock */
	rcu_periph_clock_enable(DMA_RCU);

	/* deinitialize USART TX DMA channel */
	dma_deinit(DMA_DEVICE, USART_TX_DMA_CHANNEL);

	dma_struct_para_init(&dma_init_struct);
	dma_init_struct.direction    = DMA_MEMORY_TO_PERIPHERAL;
	dma_init_struct.memory_addr  = (uint32_t)0;
	dma_init_struct.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
	dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
	dma_init_struct.number       = 0;
	dma_init_struct.periph_addr  = (uint32_t)&USART_DATA(COM_PORT);
	dma_init_struct.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
	dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
	dma_init_struct.priority     = DMA_PRIORITY_ULTRA_HIGH;
	dma_init(DMA_DEVICE, USART_TX_DMA_CHANNEL, &dma_init_struct);

	dma_circulation_disable(DMA_DEVICE, USART_TX_DMA_CHANNEL);
	dma_memory_to_memory_disable(DMA_DEVICE, USART_TX_DMA_CHANNEL);
	dma_flag_clear(DMA_DEVICE, USART_TX_DMA_CHANNEL, DMA_FLAG_G | DMA_FLAG_FTF | DMA_FLAG_HTF | DMA_FLAG_ERR);

	/* enable DMA transfer complete interrupt */
	dma_interrupt_enable(DMA_DEVICE, USART_TX_DMA_CHANNEL, DMA_INT_FTF);
	nvic_irq_enable(DMA_USART_TX_IRQ, 3, 0);

	/* enable USART transmit DMA request */
	usart_dma_transmit_config(COM_PORT, USART_TRANSMIT_DMA_ENABLE);
}


/**
 * @brief  启动一帧数据的 DMA 发送
 * @param  pucData 待发送数据首地址
 * @param  len     待发送数据长度
 * @retval TRUE: 已启动 DMA 发送
 */
BOOL xMBPortSerialDmaPut( UCHAR *pucData, USHORT len )
{
	if( (pucData == NULL) || (len == 0) ){
		return FALSE;
	}

	/* 重新装载本次发送的内存地址与长度，然后启动 DMA */
	dma_channel_disable(DMA_DEVICE, USART_TX_DMA_CHANNEL);
	dma_flag_clear(DMA_DEVICE, USART_TX_DMA_CHANNEL, DMA_FLAG_G | DMA_FLAG_FTF | DMA_FLAG_HTF | DMA_FLAG_ERR);
	dma_memory_address_config(DMA_DEVICE, USART_TX_DMA_CHANNEL, (uint32_t)pucData);
	dma_transfer_number_config(DMA_DEVICE, USART_TX_DMA_CHANNEL, len);
	dma_channel_enable(DMA_DEVICE, USART_TX_DMA_CHANNEL);

	return TRUE;
}


/**
 * @brief  发送 DMA 传输完成中断
 *         整帧数据搬移结束后通知协议栈，等价于原来的“发送寄存器空中断”
 */
void DMA_USART_TX_HANDLER(void)
{
	if(SET == dma_interrupt_flag_get(DMA_DEVICE, USART_TX_DMA_CHANNEL, DMA_INT_FLAG_FTF)){
		/* 明确清掉 FTF 标志，避免其残留导致本中断被反复触发 */
		dma_interrupt_flag_clear(DMA_DEVICE, USART_TX_DMA_CHANNEL, DMA_INT_FLAG_FTF);
		dma_channel_disable(DMA_DEVICE, USART_TX_DMA_CHANNEL);

		/* 发送完毕：通知协议栈并将 485 切回接收方向 */
		pxMBFrameCBTransmitterEmpty();
	}
}


/**
 * @brief  485 发送方向看门狗（由 Modbus 主轮询每轮调用）
 *         正常路径：DMA 完成中断 -> xMBRTUTransmitFSM -> vMBPortSerialEnable(TRUE,FALSE)
 *                   -> DELAY_TIM 中断 -> 切回接收。
 *         该中断链任一环丢失，485 会一直停在发送使能方向，导致彻底收不到数据。
 *         这里做超时兜底：发送方向停留超过 MB_485_TX_WATCHDOG_MS 就强制切回接收。
 * @retval TRUE : 检测到超时并已强制切回接收方向（调用方需同步复位协议栈发送状态）
 *         FALSE: 无需处理
 */
BOOL xMBPortSerialTxWatchdog( void )
{
	ULONG ulElapsed;
	BOOL  bRecovered = FALSE;

	if( !s_bRs485InTx ){
		return FALSE;
	}

	/* 无符号差值比较，天然兼容 systick_get_ms() 的回绕 */
	ulElapsed = (ULONG)(systick_get_ms() - s_ulTxDirEnterMs);
	if( ulElapsed < MB_485_TX_WATCHDOG_MS ){
		return FALSE;
	}

	/* 临界区保护：与 DMA / DELAY_TIM / USART 中断对方向及状态的操作互斥 */
	ENTER_CRITICAL_SECTION(  );

	if( s_bRs485InTx ){
		/* 1) 停掉尚未完成的“切回收方向”延时 */
		timer_interrupt_disable(DELAY_TIM, TIMER_INT_UP);
		timer_disable(DELAY_TIM);
		ucTxWaitCnt = 0;

		/* 2) 停掉可能卡住的发送 DMA，并清干净其标志，防止残留 FTF 再次触发中断 */
		dma_channel_disable(DMA_DEVICE, USART_TX_DMA_CHANNEL);
		dma_flag_clear(DMA_DEVICE, USART_TX_DMA_CHANNEL, DMA_FLAG_G | DMA_FLAG_FTF | DMA_FLAG_HTF | DMA_FLAG_ERR);
		dma_interrupt_flag_clear(DMA_DEVICE, USART_TX_DMA_CHANNEL, DMA_INT_FLAG_FTF);

		/* 3) 强制切回接收方向 */
		LED_485_ON;

		/* 4) 丢弃切换期间残留在接收寄存器里的无效字节 */
		if(RESET != usart_flag_get(COM_PORT, USART_FLAG_RBNE)){
			(void)USART_DATA(COM_PORT);
		}
		usart_flag_clear(COM_PORT, USART_FLAG_ORERR);
		usart_interrupt_flag_clear(COM_PORT, USART_INT_FLAG_RBNE);
		NVIC_ClearPendingIRQ(COM_PORT_IRQ);

		/* 5) 打开接收中断，恢复正常接收 */
		usart_interrupt_enable(COM_PORT, USART_INT_RBNE);

		s_bRs485InTx = FALSE;
		bRecovered = TRUE;
	}

	EXIT_CRITICAL_SECTION(  );

	return bRecovered;
}

