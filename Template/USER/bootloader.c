/*!
    \file    bootloader.c
    \brief   RS485 (Modbus RTU) bootloader 核心实现:
             flash 擦写 / CRC32 / APP 有效性检查 / 跳转 APP

    \version V1.0.0
*/

#include "bootloader.h"
#include "systick.h"
#include "bsp_usart.h"
#include "bsp_TiMbase.h"
#include "mb.h"

#include "bsp_led.h"

/* ---------------- 内部状态 ---------------- */
static bool     s_bUpdating  = 0;    /* 升级模式标志               */
static bool     s_bErased    = 0;    /* APP 区已擦除               */
static bool     s_bCmdOk     = 0;    /* 上次命令执行成功           */
static uint8_t  s_ucCrcRes   = BOOT_CRC_NOT_RUN;
static boot_err_t s_eErr     = BOOT_ERR_NONE;
static bool     s_bAppValid  = 0;

/* ---------------- flash 基本操作 ---------------- */

/*! 解锁并清除标志 */
static void flash_unlock_clear(void)
{
    fmc_unlock();
    fmc_flag_clear(FMC_FLAG_BANK0_PGERR | FMC_FLAG_BANK0_WPERR | FMC_FLAG_BANK0_END);
}

bool boot_app_is_valid(void)
{
    uint32_t sp = *(volatile uint32_t *)APP_START_ADDR;
    uint32_t pc = *(volatile uint32_t *)(APP_START_ADDR + 4U);

    /* 空 flash (0xFFFFFFFF) 直接无效 */
    if ((sp == 0xFFFFFFFFUL) || (pc == 0xFFFFFFFFUL)){
        return 0;
    }

    /* SP: 必须落在 SRAM (GD32F103RET6 = 64KB, 0x20000000 ~ 0x20010000) */
    if ((sp < 0x20000000UL) || (sp > 0x20010000UL)){
        return 0;
    }
    if (sp & 0x3UL){                       /* SP 应 4 字节对齐 */
        return 0;
    }

    /* PC: 必须落在 APP flash 区, 且为 Thumb 地址 (bit0 = 1) */
    if ((pc < APP_START_ADDR) || (pc >= (APP_START_ADDR + APP_MAX_SIZE))){
        return 0;
    }
    if ((pc & 0x1UL) == 0UL){
        return 0;
    }

    return 1;
}

void boot_init(void)
{
    s_bUpdating = 0;                 /* 每次上电/复位都清零, 不依赖 .bss 清零 */
    s_bErased   = 0;
    s_bCmdOk    = 0;
    s_ucCrcRes  = BOOT_CRC_NOT_RUN;
    s_eErr      = BOOT_ERR_NONE;
    s_bAppValid = boot_app_is_valid();
}

bool boot_is_updating(void)
{
    return s_bUpdating;
}

uint16_t boot_get_status(void)
{
    uint16_t sts = 0U;

    if (s_bAppValid){
        sts |= BOOT_STS_APP_VALID;
    }
    if (s_bUpdating){
        sts |= BOOT_STS_UPDATING;
    }
    if (s_bErased){
        sts |= BOOT_STS_ERASED;
    }
    if (s_bUpdating && s_bErased){
        sts |= BOOT_STS_WRITABLE;
    }
    if (s_bCmdOk){
        sts |= BOOT_STS_CMD_OK;
    }
    return sts;
}

void boot_set_error(boot_err_t err)
{
    s_eErr  = err;
    s_bCmdOk = 0;
}

/* ---------------- 命令实现 ---------------- */

void boot_cmd_enter_update(void)
{
    s_bUpdating = 1;
    s_bCmdOk    = 1;
    s_eErr      = BOOT_ERR_NONE;
}

/*! 按固件大小擦除 APP 区 (每页擦除期间喂狗, 防止看门狗复位) */
bool boot_cmd_erase(uint32_t fw_size)
{
    uint32_t page_cnt;
    uint32_t addr;
    fmc_state_enum st = FMC_READY;

    if (!s_bUpdating){
        boot_set_error(BOOT_ERR_NOT_UPDATE);
        return 0;
    }
    if ((fw_size == 0UL) || (fw_size > APP_MAX_SIZE)){
        boot_set_error(BOOT_ERR_BAD_SIZE);
        return 0;
    }

    page_cnt = (fw_size + 2047UL) / 2048UL;          /* 2KB / 页 */
    addr     = APP_START_ADDR;

    flash_unlock_clear();
    while (page_cnt-- > 0U){
        st = fmc_page_erase(addr);
        if (FMC_READY != st){
            break;
        }
        addr += 2048UL;
        fwdgt_counter_reload();                      /* 擦除耗时长, 逐页喂狗 */
    }
    fmc_lock();

    if (FMC_READY != st){
        boot_set_error(BOOT_ERR_NO_ERASE);
        return 0;
    }

    s_bErased = 1;
    s_bCmdOk  = 1;
    s_eErr    = BOOT_ERR_NONE;
    return 1;
}

/*! 写 APP flash: 半字对齐编程, len 必须为偶数 */
bool boot_cmd_write(uint32_t flash_addr, const uint8_t *data, uint16_t len)
{
    fmc_state_enum st = FMC_READY;

    if (!s_bUpdating){
        boot_set_error(BOOT_ERR_NOT_UPDATE);
        return 0;
    }
    if (!s_bErased){
        boot_set_error(BOOT_ERR_NOT_ERASED);
        return 0;
    }
    /* 地址范围与半字对齐检查 */
    if ((flash_addr < APP_START_ADDR) ||
        ((flash_addr + len) > (APP_START_ADDR + APP_MAX_SIZE)) ||
        (flash_addr & 0x1UL) || (len & 0x1UL)){
        boot_set_error(BOOT_ERR_BAD_ADDR);
        return 0;
    }

    flash_unlock_clear();
    while (len > 0U){
        uint16_t half = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
        st = fmc_halfword_program(flash_addr, half);
        if (FMC_READY != st){
            break;
        }
        flash_addr += 2U;
        data       += 2;
        len        -= 2U;
    }
    fmc_lock();

    if (FMC_READY != st){
        boot_set_error(BOOT_ERR_FLASH);
        return 0;
    }

    s_bCmdOk = 1;
    s_eErr   = BOOT_ERR_NONE;
    return 1;
}

/*! 对 APP 区前 fw_size 字节计算 CRC32 并与给定值比对 */
bool boot_cmd_check_crc(uint32_t fw_size, uint32_t crc32)
{
    uint32_t crc;

    if (!s_bUpdating){
        boot_set_error(BOOT_ERR_NOT_UPDATE);
        s_ucCrcRes = BOOT_CRC_FAIL;
        return 0;
    }
    if ((fw_size == 0UL) || (fw_size > APP_MAX_SIZE)){
        boot_set_error(BOOT_ERR_BAD_SIZE);
        s_ucCrcRes = BOOT_CRC_FAIL;
        return 0;
    }

    crc = boot_crc32((const uint8_t *)APP_START_ADDR, fw_size);

    if (crc == crc32){
        s_ucCrcRes = BOOT_CRC_PASS;
        s_bCmdOk   = 1;
        s_eErr     = BOOT_ERR_NONE;
        return 1;
    }

    s_ucCrcRes = BOOT_CRC_FAIL;
    boot_set_error(BOOT_ERR_CRC_FAIL);
    return 0;
}

/* ---------------- 复位/跳转 ---------------- */

void boot_system_reset(void)
{
    fmc_lock();
    NVIC_SystemReset();
}

/*! 退出 bootloader 前恢复外设到复位态, 避免 APP 收到残留中断/配置 */
static void deinit_all(void)
{
    uint32_t i;

    __disable_irq();

    /* 停止 Modbus 协议栈并复位串口/DMA/定时器 */
    (void)eMBDisable();

    usart_deinit(COM_PORT);
    dma_deinit(DMA_DEVICE, USART_TX_DMA_CHANNEL);
    dma_deinit(DMA_DEVICE, USART_RX_DMA_CHANNEL);
    timer_deinit(BASIC_TIM);
    timer_deinit(DELAY_TIM);

    /* 关闭所有 NVIC 中断使能并清 pending */
    for (i = 0U; i < 8U; i++){
        NVIC->ICER[i] = 0xFFFFFFFFUL;
        NVIC->ICPR[i] = 0xFFFFFFFFUL;
    }

    SysTick->CTRL = 0U;
    SCB->VTOR     = APP_START_ADDR;  /* 原来 0U, 改成 APP 基址 */

    /* 485 切回接收方向 (PB7 输出低, 与 portserial.c 中 LED_485_ON 一致) */
    gpio_bit_reset(GPIOB, GPIO_PIN_7);
}

bool boot_jump_to_app(void)
{
    uint32_t sp, pc;
	  volatile uint32_t i = 0;
    void   (*pReset)(void);

    if (!boot_app_is_valid()){
        boot_set_error(BOOT_ERR_APP_INVALID);
        return 0;
    }

    sp = *(volatile uint32_t *)APP_START_ADDR;
    pc = *(volatile uint32_t *)(APP_START_ADDR + 4U);
    
		/* ---- 调试标记: LED 常亮 2 秒, 明显区别于 250ms 心跳 ---- */
    LED_COM_ON;
    for (i = 0; i < 8000000UL; i++) { __NOP(); }  /* ~1s @72MHz, 不准也无所谓 */
    LED_COM_ON;
		
    deinit_all();

    __set_MSP(sp);
    __set_CONTROL(0U);
    __set_PRIMASK(0U);              /* 新增: 放开全局中断 */

    pReset = (void (*)(void))pc;
    pReset();
    return 1;
}

/* ---------------- 工具 ---------------- */

uint8_t boot_crc_result_get(void)   /* 供 user_mb_app.c 读取 HR7 */
{
    return s_ucCrcRes;
}

boot_err_t boot_error_get(void)     /* 供 user_mb_app.c 读取 HR8 */
{
    return s_eErr;
}

/*! CRC32 (IEEE 802.3 / zlib), 初值 0xFFFFFFFF, 结果异或 0xFFFFFFFF */
uint32_t boot_crc32(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFUL;
    uint32_t i;
    uint8_t  j;

    for (i = 0U; i < len; i++){
        crc ^= data[i];
        for (j = 0U; j < 8U; j++){
            crc = (crc >> 1U) ^ (0xEDB88320UL & (0U - (crc & 1UL)));
        }
    }
    return crc ^ 0xFFFFFFFFUL;
}
