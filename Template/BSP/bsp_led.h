#ifndef __LED_H
#define	__LED_H


#include "gd32f10x.h"


/* 定义LED连接的GPIO端口, 用户只需要修改下面的代码即可改变控制的LED引脚 */
// 运行灯
#define LED1_GPIO_PORT    	GPIOA			              /* GPIO端口 */
#define LED1_GPIO_CLK 	    RCU_GPIOA		/* GPIO端口时钟 */
#define LED1_GPIO_PIN		GPIO_PIN_5			        /* 连接到SCL时钟线的GPIO */


// 通讯灯
#define LED2_GPIO_PORT    	GPIOA			              /* GPIO端口 */
#define LED2_GPIO_CLK 	    RCU_GPIOA		/* GPIO端口时钟 */
#define LED2_GPIO_PIN		GPIO_PIN_4			        /* 连接到SCL时钟线的GPIO */


// 485使能
#define LED3_GPIO_PORT    	GPIOB			              /* GPIO端口 */
#define LED3_GPIO_CLK 	    RCU_GPIOB		/* GPIO端口时钟 */
#define LED3_GPIO_PIN		GPIO_PIN_7			        /* 连接到SCL时钟线的GPIO */


#define LED_RUN_ON	GPIO_BC(LED1_GPIO_PORT) = LED1_GPIO_PIN
#define LED_RUN_OFF	GPIO_BOP(LED1_GPIO_PORT) = LED1_GPIO_PIN

#define LED_COM_ON	GPIO_BC(LED2_GPIO_PORT) = LED2_GPIO_PIN
#define LED_COM_OFF	GPIO_BOP(LED2_GPIO_PORT) = LED2_GPIO_PIN


#define LED_485_ON	GPIO_BC(LED3_GPIO_PORT) = LED3_GPIO_PIN
#define LED_485_OFF	GPIO_BOP(LED3_GPIO_PORT) = LED3_GPIO_PIN


void LED_GPIO_Config(void);

#endif /* __LED_H */
