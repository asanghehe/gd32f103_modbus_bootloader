/*!
    \file    user_mb_app.h
    \brief   Modbus 从机回调 (bootloader 升级协议寄存器映射) 头文件

    \version V1.0.0
*/

#ifndef USER_MB_APP_H
#define USER_MB_APP_H

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbconfig.h"
#include "mbframe.h"

/* 保持寄存器数量: 控制区 + 数据区 (FC16 单帧最多 123 个) */
#define S_REG_HOLDING_START     0U
#define S_REG_HOLDING_NREGS     (0x1000U + 128U)     /* 控制区 0..10 + 预留, 数据区 0x1000 起 */

/* 输入/离散/线圈区不用于升级, 保持最小实现 */
#define S_REG_INPUT_START       0U
#define S_REG_INPUT_NREGS       16U
#define S_DISCRETE_INPUT_START  0U
#define S_DISCRETE_INPUT_NDISCRETES 8U
#define S_COIL_START            0U
#define S_COIL_NCOILS           8U

/* 由 main.c 查询的跳转/复位请求 (命令在协议栈回调中置位, 由主循环执行) */
BOOL    mb_jump_requested(void);
BOOL    mb_reset_requested(void);
void    mb_jump_request_clear(void);
void    mb_reset_request_clear(void);

#endif /* USER_MB_APP_H */
