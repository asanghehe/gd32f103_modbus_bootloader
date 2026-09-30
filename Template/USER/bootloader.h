/*!
    \file    bootloader.h
    \brief   RS485 (Modbus RTU) bootloader 核心模块

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
      3. APP 若启用了独立看门狗, 跳转前看门狗仍在运行, APP 必须自行重新配置并喂狗。

    升级协议 (Modbus RTU 从机, 地址 0x01, 57600 8N1, 保持寄存器):

      控制区 (FC03 读 / FC06、FC16 写):
        HR 0   (RO)  bootloader 版本, 0x0100 = V1.0
        HR 1   (RO)  状态: bit0 APP有效  bit1 升级模式  bit2 APP区已擦除
                      bit3 数据区可写(升级模式+已擦除)  bit4 上次命令执行成功
        HR 2/3 (RW)  固件总字节数 (HR2=高16位, HR3=低16位)
        HR 4/5 (RW)  固件 CRC32 (zlib/IEEE802.3, HR4=高16位, HR5=低16位)
        HR 6   (WO)  命令寄存器:
                        1 = 进入升级模式 (阻止跳转 APP)
                        2 = 擦除 APP 区 (按 HR2/3 的大小, 阻塞执行约 40ms/页)
                        3 = 校验 CRC32 (对 APP 区前 N 字节计算并与 HR4/5 比对)
                        4 = 跳转到 APP (要求 APP 有效)
                        5 = 系统复位
        HR 7   (RO)  CRC 校验结果: 0=未校验 1=通过 2=失败
        HR 8   (RO)  最近一次错误码 (见 boot_err_t)
        HR 9/10(RO)  APP 起始地址 (0x08008000), 便于主机核对布局

      数据区 (FC16 写, FC03 读):
        HR 0x1000 (4096) 起映射到 APP Flash:
            flash 地址 = 0x08008000 + (寄存器地址 - 4096) * 2
        单帧最多写 123 个寄存器 (246 字节), 最大固件 120KB。
        主机按顺序分块写入即可, 也可以任意顺序/重写。

      典型升级流程:
        1. 设备上电, bootloader 在 BOOT_WAIT_MS 时间内等待 "进入升级" 命令
           (主机复位设备后立即发: FC06 写 HR6=1);
           若超时且 APP 有效则自动跳转 APP, APP 无效则常驻升级模式。
        2. 写 HR2/3 = 固件大小, HR4/5 = 固件 CRC32;
        3. 写 HR6=2 擦除 APP 区 (响应可能需要数秒, 主机超时应 >= 10s);
        4. 循环 FC16 写数据区 0x1000 起, 每帧 <= 123 寄存器;
        5. 写 HR6=3 校验, 读 HR7 确认 = 1;
        6. 写 HR6=4 跳转到新 APP。
*/

#ifndef BOOTLOADER_H
#define BOOTLOADER_H

#include "gd32f10x.h"

/* ---------------- 内存布局 ---------------- */
#define BOOT_START_ADDR         0x08000000UL     /* bootloader 起始地址            */
#define BOOT_MAX_SIZE           0x8000UL         /* bootloader 区域 32KB           */
#define APP_START_ADDR          0x08008000UL     /* APP 起始地址                   */
#define APP_MAX_SIZE            0x1E000UL        /* 协议支持的最大固件 120KB       */

/* ---------------- Modbus 寄存器映射 ---------------- */
#define MB_REG_BOOT_VER         0U               /* HR0  版本 (RO)                 */
#define MB_REG_STATUS           1U               /* HR1  状态 (RO)                 */
#define MB_REG_FW_SIZE_H        2U               /* HR2  固件大小高16位 (RW)       */
#define MB_REG_FW_SIZE_L        3U               /* HR3  固件大小低16位 (RW)       */
#define MB_REG_FW_CRC_H         4U               /* HR4  CRC32 高16位 (RW)         */
#define MB_REG_FW_CRC_L         5U               /* HR5  CRC32 低16位 (RW)         */
#define MB_REG_CMD              6U               /* HR6  命令 (WO)                 */
#define MB_REG_CRC_RESULT       7U               /* HR7  CRC 校验结果 (RO)         */
#define MB_REG_ERRCODE          8U               /* HR8  错误码 (RO)               */
#define MB_REG_APP_ADDR_H       9U               /* HR9  APP 地址高16位 (RO)       */
#define MB_REG_APP_ADDR_L       10U              /* HR10 APP 地址低16位 (RO)       */
#define MB_CTRL_REG_NUM         11U              /* 控制区寄存器个数               */

#define MB_DATA_REG_BASE        0x1000U          /* 数据区起始寄存器地址           */

/* HR1 状态位 */
#define BOOT_STS_APP_VALID      0x0001U          /* APP 向量有效                   */
#define BOOT_STS_UPDATING       0x0002U          /* 处于升级模式                   */
#define BOOT_STS_ERASED         0x0004U          /* APP 区已擦除                   */
#define BOOT_STS_WRITABLE       0x0008U          /* 数据区可写                     */
#define BOOT_STS_CMD_OK         0x0010U          /* 上次命令执行成功               */

/* HR6 命令值 */
#define BOOT_CMD_ENTER_UPDATE   1U
#define BOOT_CMD_ERASE          2U
#define BOOT_CMD_CHECK_CRC      3U
#define BOOT_CMD_JUMP           4U
#define BOOT_CMD_RESET          5U

/* HR8 错误码 */
typedef enum
{
    BOOT_ERR_NONE          = 0,
    BOOT_ERR_NOT_UPDATE    = 1,     /* 未进入升级模式就执行升级命令     */
    BOOT_ERR_NOT_ERASED    = 2,     /* 写数据前未擦除                   */
    BOOT_ERR_BAD_SIZE      = 3,     /* 固件大小非法                     */
    BOOT_ERR_BAD_ADDR      = 4,     /* 访问了越界/非法的寄存器地址      */
    BOOT_ERR_FLASH         = 5,     /* flash 擦写失败                   */
    BOOT_ERR_CRC_FAIL      = 6,     /* CRC 校验失败                     */
    BOOT_ERR_APP_INVALID   = 7,     /* APP 无效, 无法跳转               */
    BOOT_ERR_NO_ERASE      = 8,     /* 擦除失败                         */
    BOOT_ERR_BUSY          = 9,     /* 内部状态冲突                     */
} boot_err_t;

/* CRC 校验结果 */
#define BOOT_CRC_NOT_RUN        0U
#define BOOT_CRC_PASS           1U
#define BOOT_CRC_FAIL           2U

/* bootloader 常量 */
#define BOOT_SLAVE_ADDR         0x01U            /* Modbus 从机地址                */
#define BOOT_BAUDRATE           57600UL          /* RS485 波特率                   */
#define BOOT_WAIT_MS            2000UL           /* 上电等待升级命令的时间窗口     */

/* ---------------- 对外接口 ---------------- */
void    boot_init(void);                         /* 初始化状态(校验 APP 有效性)    */
bool    boot_app_is_valid(void);                 /* APP 向量是否有效               */
bool    boot_is_updating(void);                  /* 是否处于升级模式               */
uint16_t boot_get_status(void);                  /* HR1 状态字                     */
void    boot_set_error(boot_err_t err);          /* 记录错误码                     */

void    boot_cmd_enter_update(void);             /* 进入升级模式                   */
bool    boot_cmd_erase(uint32_t fw_size);        /* 擦除 APP 区, true=成功         */
bool    boot_cmd_write(uint32_t flash_addr, const uint8_t *data, uint16_t len);
                                                 /* 写 APP flash (半字对齐)        */
bool    boot_cmd_check_crc(uint32_t fw_size, uint32_t crc32);  /* 校验 CRC, 结果存内部 */
bool    boot_jump_to_app(void);                  /* 跳转 APP (成功不返回)          */
void    boot_system_reset(void);                 /* 系统复位                       */

uint8_t boot_crc_result_get(void);              /* HR7 CRC 结果                  */
boot_err_t boot_error_get(void);                /* HR8 错误码                    */
uint32_t boot_crc32(const uint8_t *data, uint32_t len);  /* CRC32 (zlib)            */

#endif /* BOOTLOADER_H */
