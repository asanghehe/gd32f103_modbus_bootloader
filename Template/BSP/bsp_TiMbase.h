#ifndef __BSP_TIMEBASE_H
#define __BSP_TIMEBASE_H


#include "gd32f10x.h"


/********************基本定时器TIM参数定义，只限TIM6、7************/
#define BASIC_TIM6 // 如果使用TIM7，注释掉这个宏即可

#ifdef  BASIC_TIM6 // 使用基本定时器TIM6
#define            BASIC_TIM                   TIMER6
#define            BASIC_TIM_CLK               RCU_TIMER6

#define            DELAY_TIM                   TIMER5
#define            DELAY_TIM_CLK               RCU_TIMER5
#define            DELAY_TIM_IRQ               TIMER5_IRQn
#define            DELAY_TIM_IRQHandler        TIMER5_IRQHandler

#define            BASIC_TIM_Period            50-1
#define            BASIC_TIM_IRQ               TIMER6_IRQn
#define            BASIC_TIM_IRQHandler        TIMER6_IRQHandler

#else  // 使用基本定时器TIM7
#define            BASIC_TIM                   TIM7
#define            BASIC_TIM_APBxClock_FUN     RCC_APB1PeriphClockCmd
#define            BASIC_TIM_CLK               RCC_APB1Periph_TIM7
#define            BASIC_TIM_Period            1000-1
#define            BASIC_TIM_Prescaler         71
#define            BASIC_TIM_IRQ               TIM7_IRQn
#define            BASIC_TIM_IRQHandler        TIM7_IRQHandler

#endif
/**************************函数声明********************************/

void BASIC_TIM_Init(void);


#endif	/* __BSP_TIMEBASE_H */


