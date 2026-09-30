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
 * File: $Id: porttimer.c,v 1.60 2013/08/13 15:07:05 Armink $
 */

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"


/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "bsp_TiMbase.h" 

/* ----------------------- static functions ---------------------------------*/
static USHORT           timeout     = 0;
static USHORT           downcounter = 0;

static void prvvTIMERExpiredISR(void);

/* ----------------------- Start implementation -----------------------------*/
BOOL xMBPortTimersInit(USHORT usTim1Timerout50us)
{
    timer_parameter_struct timer_initpara;
	
    timeout = usTim1Timerout50us;
	
    rcu_periph_clock_enable(BASIC_TIM_CLK);
	
    timer_deinit(BASIC_TIM);

    timer_struct_para_init(&timer_initpara);
    timer_initpara.prescaler         = SystemCoreClock / 1000000 - 1;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 50 - 1;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_init(BASIC_TIM, &timer_initpara);

    //timer_interrupt_flag_clear(BASIC_TIM, TIMER_INT_FLAG_UP);
    //timer_interrupt_enable(BASIC_TIM, TIMER_INT_UP);
		
    //timer_enable(BASIC_TIM);
	  
		nvic_irq_enable(BASIC_TIM_IRQ, 4, 1);
		NVIC_DisableIRQ(BASIC_TIM_IRQ);
	
	
		//延时开启接收定时器
    rcu_periph_clock_enable(DELAY_TIM_CLK);
	
    /* prescaler for 1us tick */
    timer_initpara.prescaler         = SystemCoreClock / 1000000 - 1;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 299;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(DELAY_TIM, &timer_initpara);
		
		timer_single_pulse_mode_config(DELAY_TIM, TIMER_SP_MODE_SINGLE);
		
    nvic_irq_enable(DELAY_TIM_IRQ, 5, 1);
		
		
    return TRUE;
}

void vMBPortTimersEnable()
{
		downcounter = timeout;
		// 清除计数器中断标志位
    timer_interrupt_flag_clear(BASIC_TIM, TIMER_INT_FLAG_UP);
		timer_interrupt_enable(BASIC_TIM, TIMER_INT_UP);
		timer_enable(BASIC_TIM);
	
		NVIC_EnableIRQ(BASIC_TIM_IRQ);
}

void vMBPortTimersDisable()
{
		//close计数器中断
    timer_interrupt_disable(BASIC_TIM, TIMER_INT_UP);
		timer_disable(BASIC_TIM);
	
		NVIC_DisableIRQ(BASIC_TIM_IRQ);
}

void prvvTIMERExpiredISR(void)
{
    (void) pxMBPortCBTimerExpired();
}


/**
 * @brief This function handles TIM7 global interrupt.
 */
void BASIC_TIM_IRQHandler( void )
{
		if(SET == timer_interrupt_flag_get(BASIC_TIM, TIMER_INT_FLAG_UP)){
        /* clear channel 0 interrupt bit */
        timer_interrupt_flag_clear(BASIC_TIM, TIMER_INT_FLAG_UP);
        
				 /* Decrement down-counter and check if reached zero */
        if (--downcounter == 0) {
            /* Timer expired, call the callback function */
            //vMBTimerDebugSetLow();
            pxMBPortCBTimerExpired();
        }
    }
}
