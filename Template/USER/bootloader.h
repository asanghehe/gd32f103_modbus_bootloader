/*!
    \file    bootloader.h
    \brief   RS485 simple-frame bootloader 核心模块

    \version V1.0.0
*/

/*
    内存布局 (GD32F103, 页大小 2KB):
    ----------------------------------------------------------------
    | 0x08000000 ~ 0x08007FFF |  32KB | bootloader (本工程, IROM 限制 32KB) |
    | 0x08008000 ~ ...        |  最多 120KB | 应用程序 (APP)                  |
    ----------------------------------------------------------------

    APP 工程需要注意:
      1. 链接地址 (IROM 起始) 必须改为 0x08008000;
      2. APP 里需重定位中断向量表: SCB->VTOR = 0x08008000
         (system_gd32f10x.c 中设置 VECT_TAB_OFFSET = 0x8000 即可);
      3. bootloader 不启动或喂独立看门狗。若 APP 启用了独立看门狗, 系统复位后
         看门狗仍会运行, bootloader 不负责维护; APP 应自行维护, 否则可能反复复位。

    镜像 CRC32 元数据保存在 APP 区之后的独立 Flash 页, 跳转前重新校验整个镜像。
*/

#ifndef BOOTLOADER_H
#define BOOTLOADER_H

#include "gd32f10x.h"

/* ---------------- 内存布局 ---------------- */
#define BOOT_START_ADDR         0x08000000UL     /* bootloader 起始地址            */
#define BOOT_MAX_SIZE           0x8000UL         /* bootloader 区域 32KB           */
#define APP_START_ADDR          0x08008000UL     /* APP 起始地址                   */
#define APP_MAX_SIZE            0x1E000UL        /* APP 最大固件 120KB              */
#define BOOT_META_ADDR          (APP_START_ADDR + APP_MAX_SIZE)
#define BOOT_FLASH_PAGE_SIZE    2048UL
#define BOOT_VERSION            0x0200U

/* boot status bits */
#define BOOT_STS_APP_VALID      0x0001U
#define BOOT_STS_UPDATING       0x0002U
#define BOOT_STS_ERASED         0x0004U
#define BOOT_STS_WRITABLE       0x0008U
#define BOOT_STS_CMD_OK         0x0010U

/* 命令错误码 */
typedef enum
{
    BOOT_ERR_NONE          = 0,
    BOOT_ERR_NOT_UPDATE    = 1,     /* 未执行 BEGIN 就写入或校验         */
    BOOT_ERR_NOT_ERASED    = 2,     /* APP 区未擦除                       */
    BOOT_ERR_BAD_SIZE      = 3,     /* 固件大小非法                     */
    BOOT_ERR_BAD_ADDR      = 4,     /* 地址或偏移越界                     */
    BOOT_ERR_FLASH         = 5,     /* flash 擦写失败                   */
    BOOT_ERR_CRC_FAIL      = 6,     /* CRC 校验失败                     */
    BOOT_ERR_APP_INVALID   = 7,     /* APP 无效, 无法跳转               */
    BOOT_ERR_NO_ERASE      = 8,     /* 擦除失败                         */
    BOOT_ERR_BUSY          = 9,     /* 内部状态冲突                     */
} boot_err_t;

/* bootloader 常量 */
#define BOOT_BAUDRATE           57600UL          /* RS485 波特率                   */
#define BOOT_WAIT_MS            2000UL           /* 上电等待升级命令的时间窗口     */

/* ---------------- 对外接口 ---------------- */
void    boot_init(void);                         /* 初始化状态(校验 APP 有效性)    */
bool    boot_app_is_valid(void);                 /* APP 向量与 CRC32 是否有效      */
bool    boot_is_updating(void);                  /* 是否处于升级模式               */
uint16_t boot_get_status(void);                  /* 状态字                         */
void    boot_set_error(boot_err_t err);          /* 记录错误码                     */

bool    boot_cmd_begin_update(uint32_t fw_size, uint32_t crc32);
bool    boot_cmd_write(uint32_t offset, const uint8_t *data, uint16_t len);
bool    boot_cmd_check_crc(void);
bool    boot_jump_to_app(void);                  /* 跳转 APP (成功不返回)          */
void    boot_system_reset(void);                 /* 系统复位                       */

uint32_t boot_get_app_size(void);
uint32_t boot_get_app_crc(void);
boot_err_t boot_get_error(void);
uint32_t boot_crc32(const uint8_t *data, uint32_t len);  /* CRC32 (zlib)            */

#endif /* BOOTLOADER_H */
