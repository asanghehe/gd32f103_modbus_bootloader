/*!
    \file    main.c
    \brief   RS485 simple-frame bootloader 主程序

    \version V1.0.0
*/

#include "gd32f10x.h"
#include "gd32f10x_libopt.h"
#include "systick.h"
#include "main.h"

#include "bsp_led.h"

#include "bootloader.h"
#include "boot_protocol.h"

static void led_heartbeat(void);

/*!
    \brief      main function
*/
int main(void)
{
    uint32_t ulBootStart;

    nvic_priority_group_set(NVIC_PRIGROUP_PRE3_SUB1);
    SystemInit();
    systick_config();

    /* LED + 485 方向控制引脚初始化 */
    LED_GPIO_Config();

    /* 校验 APP 区向量, 记录有效性 */
    boot_init();

    ulBootStart = systick_get_ms();

    boot_protocol_init(BOOT_BAUDRATE);

    while(1){
        boot_protocol_poll();

        led_heartbeat();

        /* 上电等待窗口结束: 未进入升级模式且 APP 有效则自动跳转 */
        if(!boot_is_updating() && boot_app_is_valid()){
            if((systick_get_ms() - ulBootStart) > BOOT_WAIT_MS){
                (void)boot_jump_to_app();
            }
        }
    }
}

/*!
    \brief      运行灯心跳: 250ms 翻转一次, 指示 bootloader 存活
*/
static void led_heartbeat(void)
{
    static uint32_t s_ulLast = 0U;
    static uint8_t  s_ucOn = 0U;
    uint32_t ulNow = systick_get_ms();

    if((ulNow - s_ulLast) >= 250U){
        s_ulLast = ulNow;
        s_ucOn = (uint8_t)!s_ucOn;
        if(s_ucOn){
            LED_RUN_ON;
        }else{
            LED_RUN_OFF;
        }
    }
}
