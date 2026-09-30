/*!
    \file    user_mb_app.c
    \brief   Modbus 从机回调: bootloader 升级协议寄存器映射
             (协议详见 bootloader.h 文件头注释)

    \version V1.0.0
*/

#include "user_mb_app.h"
#include "bootloader.h"
#include <string.h>

/* ------------------------Slave mode use these variables----------------------*/

/* 输入/离散/线圈缓冲区 (bootloader 不使用, 仅为通过 FreeModbus 注册回调而保留) */
USHORT   usSRegInStart  = S_REG_INPUT_START;
USHORT   usSRegInBuf[S_REG_INPUT_NREGS];
USHORT   usSCoilStart   = S_COIL_START;
#if S_COIL_NCOILS % 8
UCHAR    ucSCoilBuf[S_COIL_NCOILS / 8 + 1];
#else
UCHAR    ucSCoilBuf[S_COIL_NCOILS / 8];
#endif
USHORT   usSDiscInStart = S_DISCRETE_INPUT_START;
#if S_DISCRETE_INPUT_NDISCRETES % 8
UCHAR    ucSDiscInBuf[S_DISCRETE_INPUT_NDISCRETES / 8 + 1];
#else
UCHAR    ucSDiscInBuf[S_DISCRETE_INPUT_NDISCRETES / 8];
#endif

/* 升级控制参数 (由主机通过保持寄存器写入) */
static uint32_t s_ulFwSize = 0UL;      /* HR2/3 固件大小       */
static uint32_t s_ulFwCrc  = 0UL;      /* HR4/5 固件 CRC32     */

/* 供 main.c 查询: 收到跳转/复位命令后由主循环执行 (不能在协议栈回调里直接跳) */
static volatile BOOL s_bJumpReq   = FALSE;
static volatile BOOL s_bResetReq  = FALSE;

BOOL mb_jump_requested(void)  { return s_bJumpReq;  }
BOOL mb_reset_requested(void) { return s_bResetReq; }
void mb_jump_request_clear(void)  { s_bJumpReq  = FALSE; }
void mb_reset_request_clear(void) { s_bResetReq = FALSE; }

/*!
    \brief      处理主机写入的命令 (HR6)
*/
static void mb_handle_cmd(UCHAR ucCmd)
{
    switch (ucCmd){
    case BOOT_CMD_ENTER_UPDATE:
        boot_cmd_enter_update();
        break;

    case BOOT_CMD_ERASE:
        /* 阻塞执行, 页擦除约 40ms/页, 主机响应超时应 >= 10s */
        (void)boot_cmd_erase(s_ulFwSize);
        break;

    case BOOT_CMD_CHECK_CRC:
        (void)boot_cmd_check_crc(s_ulFwSize, s_ulFwCrc);
        break;

    case BOOT_CMD_JUMP:
        s_bJumpReq = TRUE;      /* 由主循环执行, 确保响应帧发送完成 */
        break;

    case BOOT_CMD_RESET:
        s_bResetReq = TRUE;     /* 由主循环执行, 确保响应帧发送完成 */
        break;

    default:
        boot_set_error(BOOT_ERR_BAD_ADDR);
        break;
    }
}

/*!
    \brief      保持寄存器回调: 控制区 0..10 + 数据区 0x1000 起映射 APP flash
*/
eMBErrorCode eMBRegHoldingCB(UCHAR *pucRegBuffer, USHORT usAddress,
                             USHORT usNRegs, eMBRegisterMode eMode)
{
    USHORT usRegIndex;

    /* FreeModbus 已保证 usAddress >= 1, 转为 0 基地址 */
    usRegIndex = (USHORT)(usAddress - 1U);

    if (usRegIndex >= MB_DATA_REG_BASE){
        /* ---------- 数据区: 直接映射 APP flash ---------- */
        uint32_t flash_addr = APP_START_ADDR +
                              (uint32_t)(usRegIndex - MB_DATA_REG_BASE) * 2UL;
        USHORT i;

        if (eMode == MB_REG_WRITE){
            /* FC06/FC16 写: 逐半字编程 (寄存器数据为大端字节序) */
            if (!boot_cmd_write(flash_addr, pucRegBuffer, (uint16_t)(usNRegs * 2U))){
                return MB_ENOREG;   /* 内部会记录具体错误码 */
            }
        }else{
            /* FC03 读: 直接读 flash, 转大端 */
            for (i = 0U; i < usNRegs; i++){
                uint16_t val = *(volatile uint16_t *)(flash_addr + i * 2UL);
                *pucRegBuffer++ = (UCHAR)(val >> 8);
                *pucRegBuffer++ = (UCHAR)(val & 0xFFU);
            }
        }
        return MB_ENOERR;
    }

    /* ---------- 控制区 ---------- */
    if ((usRegIndex + usNRegs) > MB_CTRL_REG_NUM){
        return MB_ENOREG;
    }

    if (eMode == MB_REG_WRITE){
        USHORT i;
        for (i = 0U; i < usNRegs; i++){
            USHORT val = (USHORT)(((uint16_t)pucRegBuffer[2U * i] << 8) |
                                  pucRegBuffer[2U * i + 1U]);
            switch ((uint16_t)(usRegIndex + i)){
            case MB_REG_FW_SIZE_H:
                s_ulFwSize = (s_ulFwSize & 0x0000FFFFUL) | ((uint32_t)val << 16);
                break;
            case MB_REG_FW_SIZE_L:
                s_ulFwSize = (s_ulFwSize & 0xFFFF0000UL) | val;
                break;
            case MB_REG_FW_CRC_H:
                s_ulFwCrc = (s_ulFwCrc & 0x0000FFFFUL) | ((uint32_t)val << 16);
                break;
            case MB_REG_FW_CRC_L:
                s_ulFwCrc = (s_ulFwCrc & 0xFFFF0000UL) | val;
                break;
            case MB_REG_CMD:
                mb_handle_cmd((UCHAR)val);
                break;
            default:
                /* 只读寄存器 (版本/状态/结果) 不允许写 */
                return MB_ENOREG;
            }
        }
    }else{
        USHORT i;
        for (i = 0U; i < usNRegs; i++){
            uint16_t val;
            switch ((uint16_t)(usRegIndex + i)){
            case MB_REG_BOOT_VER:   val = 0x0100U;                          break;
            case MB_REG_STATUS:     val = boot_get_status();                break;
            case MB_REG_FW_SIZE_H:  val = (uint16_t)(s_ulFwSize >> 16);     break;
            case MB_REG_FW_SIZE_L:  val = (uint16_t)s_ulFwSize;             break;
            case MB_REG_FW_CRC_H:   val = (uint16_t)(s_ulFwCrc >> 16);      break;
            case MB_REG_FW_CRC_L:   val = (uint16_t)s_ulFwCrc;              break;
            case MB_REG_CMD:        val = 0U;                               break;
            case MB_REG_CRC_RESULT: val = (uint16_t)boot_crc_result_get();  break;
            case MB_REG_ERRCODE:    val = (uint16_t)boot_error_get();       break;
            case MB_REG_APP_ADDR_H: val = (uint16_t)(APP_START_ADDR >> 16); break;
            case MB_REG_APP_ADDR_L: val = (uint16_t)APP_START_ADDR;         break;
            default:                val = 0U;                               break;
            }
            *pucRegBuffer++ = (UCHAR)(val >> 8);
            *pucRegBuffer++ = (UCHAR)(val & 0xFFU);
        }
    }

    return MB_ENOERR;
}

/*!
    \brief      输入寄存器回调 (升级协议未使用)
*/
eMBErrorCode eMBRegInputCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs)
{
    (void)usAddress;
    (void)usNRegs;
    (void)pucRegBuffer;
    return MB_ENOREG;
}

/*!
    \brief      线圈回调 (升级协议未使用)
*/
eMBErrorCode eMBRegCoilsCB(UCHAR *pucRegBuffer, USHORT usAddress,
                           USHORT usNCoils, eMBRegisterMode eMode)
{
    (void)pucRegBuffer;
    (void)usAddress;
    (void)usNCoils;
    (void)eMode;
    return MB_ENOREG;
}

/*!
    \brief      离散输入回调 (升级协议未使用)
*/
eMBErrorCode eMBRegDiscreteCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNDiscrete)
{
    (void)pucRegBuffer;
    (void)usAddress;
    (void)usNDiscrete;
    return MB_ENOREG;
}
