/**
  ******************************************************************************
  * @file    bsp_led.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   led应用函数接口
  ******************************************************************************
  * @attention
  *
  * 实验平台:秉火 F103-霸道 STM32 开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */
  
#include "bsp_led.h"   

 /**
  * @brief  初始化控制LED的IO
  * @param  无
  * @retval 无
  */
void LED_GPIO_Config(void)
{		
	
    /* enable the led clock */
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
	
    /* configure led GPIO port */ 
    gpio_init(LED1_GPIO_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ, LED1_GPIO_PIN);
    gpio_init(LED2_GPIO_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ, LED2_GPIO_PIN);
    gpio_init(LED3_GPIO_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, LED3_GPIO_PIN);


    LED_RUN_ON;
		LED_COM_OFF;
		LED_485_ON;
}



void gd_led_on(uint32_t gpio_periph, uint32_t pin)
{
    GPIO_BOP(gpio_periph) = pin;
}


void gd_led_off(uint32_t gpio_periph, uint32_t pin)
{
    GPIO_BC(gpio_periph) = pin;
}


void gd_led_toggle(uint32_t gpio_periph, uint32_t pin)
{
    gpio_bit_write(gpio_periph, pin, 
        (bit_status)(1-gpio_input_bit_get(gpio_periph, pin)));
}
/*********************************************END OF FILE**********************/
