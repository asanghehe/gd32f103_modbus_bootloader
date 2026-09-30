/*!
    \file    main.c
    \brief   RS485 (Modbus RTU) bootloader 主程序

    \version V1.0.0
*/

#include "gd32f10x.h"
#include "systick.h"
#include "main.h"

#include "bsp_led.h"

#include "mb.h"
#include "user_mb_app.h"
#include "bootloader.h"

static void wdg_init(void);
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

    wdg_init();

    ulBootStart = systick_get_ms();

    /* Modbus RTU 从机: 地址 0x01, 57600 8N1 (RS485) */
    (void)eMBInit(MB_RTU, BOOT_SLAVE_ADDR, 0, BOOT_BAUDRATE, MB_PAR_NONE, 1);
    (void)eMBEnable();

    while(1){
        (void)eMBPoll();

        led_heartbeat();
		
        /* 收到复位命令: 等响应帧发完再复位 */
        if(mb_reset_requested()){
            delay_1ms(50);
            boot_system_reset();
        }

        /* 收到跳转命令: 等响应帧发完再跳转 */
        if(mb_jump_requested()){
            delay_1ms(50);
            mb_jump_request_clear();
            (void)boot_jump_to_app();
            /* 跳转失败则继续留在升级模式 */
        }

        /* 上电等待窗口结束: 未进入升级模式且 APP 有效则自动跳转 */
        if(!boot_is_updating() && boot_app_is_valid()){
            if((systick_get_ms() - ulBootStart) > BOOT_WAIT_MS){
                (void)boot_jump_to_app();
            }
        }

        /* reload FWDGT counter */
        fwdgt_counter_reload();
    }
}

/*!
    \brief      独立看门狗: IRC40K/64 = 625Hz, 500 计数约 800ms
                (擦除命令在协议栈回调内逐页喂狗, 不会超时)
*/
static void wdg_init(void)
{
    rcu_all_reset_flag_clear();
    fwdgt_config(500, FWDGT_PSC_DIV64);
    fwdgt_enable();
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
