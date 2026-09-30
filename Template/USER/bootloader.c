/*!
    \file    bootloader.c
    \brief   RS485 simple-frame bootloader 核心实现:
             flash 擦写 / CRC32 / APP 有效性检查 / 跳转 APP

    \version V1.0.0
*/

#include "bootloader.h"
#include "systick.h"
#include "bsp_usart.h"
#include "gd32f10x_libopt.h"

#include "bsp_led.h"

/* ---------------- 内部状态 ---------------- */
static bool     s_bUpdating  = 0;    /* 升级模式标志               */
static bool     s_bErased    = 0;    /* APP 区已擦除               */
static bool     s_bCmdOk     = 0;    /* 上次命令执行成功           */
static boot_err_t s_eErr     = BOOT_ERR_NONE;
static bool     s_bAppValid  = 0;
static uint32_t s_ulFwSize   = 0U;
static uint32_t s_ulFwCrc    = 0U;

#define BOOT_META_MAGIC      0x424F4F54UL

typedef struct
{
    uint32_t magic;
    uint32_t size;
    uint32_t crc;
    uint32_t magic_inv;
    uint32_t size_inv;
    uint32_t crc_inv;
} boot_meta_t;

/* ---------------- flash 基本操作 ---------------- */

/*! 解锁并清除标志 */
static void flash_unlock_clear(void)
{
    fmc_unlock();
    fmc_flag_clear(FMC_FLAG_BANK0_PGERR | FMC_FLAG_BANK0_WPERR | FMC_FLAG_BANK0_END);
}

static bool boot_vector_is_valid(uint32_t fw_size)
{
    uint32_t sp = *(volatile uint32_t *)APP_START_ADDR;
    uint32_t pc = *(volatile uint32_t *)(APP_START_ADDR + 4U);

    if ((fw_size < 8UL) || (fw_size > APP_MAX_SIZE)){
        return 0;
    }

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
    if ((pc < APP_START_ADDR) || (pc >= (APP_START_ADDR + fw_size))){
        return 0;
    }
    if ((pc & 0x1UL) == 0UL){
        return 0;
    }

    return 1;
}

static bool boot_app_verify(void)
{
    const boot_meta_t *meta = (const boot_meta_t *)BOOT_META_ADDR;

    if ((meta->magic != BOOT_META_MAGIC) ||
        (meta->magic_inv != ~BOOT_META_MAGIC) ||
        (meta->size == 0UL) || (meta->size > APP_MAX_SIZE) ||
        (meta->size_inv != ~meta->size) ||
        (meta->crc_inv != ~meta->crc)){
        return 0;
    }
    if (!boot_vector_is_valid(meta->size)){
        return 0;
    }

    return (boot_crc32((const uint8_t *)APP_START_ADDR, meta->size) == meta->crc);
}

static bool boot_meta_write(uint32_t fw_size, uint32_t fw_crc)
{
    boot_meta_t meta;
    uint32_t words[6];
    uint32_t index;
    fmc_state_enum state = FMC_READY;

    meta.magic = BOOT_META_MAGIC;
    meta.size = fw_size;
    meta.crc = fw_crc;
    meta.magic_inv = ~BOOT_META_MAGIC;
    meta.size_inv = ~fw_size;
    meta.crc_inv = ~fw_crc;
    words[0] = meta.magic;
    words[1] = meta.size;
    words[2] = meta.crc;
    words[3] = meta.magic_inv;
    words[4] = meta.size_inv;
    words[5] = meta.crc_inv;

    flash_unlock_clear();
    for (index = 0U; index < 12U; index++){
        uint16_t halfword = (uint16_t)(words[index / 2U] >> ((index & 1U) * 16U));
        state = fmc_halfword_program(BOOT_META_ADDR + index * 2U, halfword);
        if (state != FMC_READY){
            break;
        }
    }
    fmc_lock();

    return (state == FMC_READY);
}

bool boot_app_is_valid(void)
{
    return s_bAppValid;
}

void boot_init(void)
{
    s_bUpdating = 0;                 /* 每次上电/复位都清零, 不依赖 .bss 清零 */
    s_bErased   = 0;
    s_bCmdOk    = 0;
    s_eErr      = BOOT_ERR_NONE;
    s_ulFwSize  = 0U;
    s_ulFwCrc   = 0U;
    s_bAppValid = boot_app_verify();
    if (s_bAppValid){
        const boot_meta_t *meta = (const boot_meta_t *)BOOT_META_ADDR;
        s_ulFwSize = meta->size;
        s_ulFwCrc  = meta->crc;
    }
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

bool boot_cmd_begin_update(uint32_t fw_size, uint32_t crc32)
{
    uint32_t page_count;
    uint32_t address;
    fmc_state_enum state;

    if ((fw_size == 0UL) || (fw_size > APP_MAX_SIZE)){
        boot_set_error(BOOT_ERR_BAD_SIZE);
        return 0;
    }

    s_bUpdating = 1;
    s_bErased = 0;
    s_bAppValid = 0;
    s_ulFwSize = fw_size;
    s_ulFwCrc = crc32;

    flash_unlock_clear();
    state = fmc_page_erase(BOOT_META_ADDR);
    fmc_lock();
    if (state != FMC_READY){
        boot_set_error(BOOT_ERR_NO_ERASE);
        return 0;
    }

    page_count = (fw_size + BOOT_FLASH_PAGE_SIZE - 1UL) / BOOT_FLASH_PAGE_SIZE;
    address = APP_START_ADDR;
    flash_unlock_clear();
    while (page_count-- > 0U){
        state = fmc_page_erase(address);
        if (state != FMC_READY){
            break;
        }
        address += BOOT_FLASH_PAGE_SIZE;
    }
    fmc_lock();

    if (state != FMC_READY){
        boot_set_error(BOOT_ERR_NO_ERASE);
        return 0;
    }

    s_bErased = 1;
    s_bCmdOk  = 1;
    s_eErr    = BOOT_ERR_NONE;
    return 1;
}

/*! 写 APP flash: 半字对齐编程, 奇数字节仅允许出现在最后一块 */
bool boot_cmd_write(uint32_t offset, const uint8_t *data, uint16_t len)
{
    uint16_t index = 0U;
    fmc_state_enum state = FMC_READY;

    if (!s_bUpdating){
        boot_set_error(BOOT_ERR_NOT_UPDATE);
        return 0;
    }
    if (!s_bErased){
        boot_set_error(BOOT_ERR_NOT_ERASED);
        return 0;
    }
    if ((len == 0U) || (offset > s_ulFwSize) ||
        ((uint32_t)len > (s_ulFwSize - offset)) || (offset & 1UL) ||
        ((len & 1U) && ((uint32_t)len != (s_ulFwSize - offset)))){
        boot_set_error(BOOT_ERR_BAD_ADDR);
        return 0;
    }

    flash_unlock_clear();
    while (index < len){
        uint16_t half = data[index];
        if ((uint16_t)(index + 1U) < len){
            half |= (uint16_t)data[index + 1U] << 8;
        }else{
            half |= 0xFF00U;
        }
        if (*(volatile uint16_t *)(APP_START_ADDR + offset + index) == half){
            index += 2U;
            continue;
        }
        state = fmc_halfword_program(APP_START_ADDR + offset + index, half);
        if (state != FMC_READY){
            break;
        }
        index += 2U;
    }
    fmc_lock();

    if (state != FMC_READY){
        boot_set_error(BOOT_ERR_FLASH);
        return 0;
    }

    s_bCmdOk = 1;
    s_eErr   = BOOT_ERR_NONE;
    return 1;
}

/*! 校验 APP 镜像并提交可启动元数据 */
bool boot_cmd_check_crc(void)
{
    if (!s_bUpdating){
        boot_set_error(BOOT_ERR_NOT_UPDATE);
        return 0;
    }
    if ((s_ulFwSize == 0UL) || (s_ulFwSize > APP_MAX_SIZE)){
        boot_set_error(BOOT_ERR_BAD_SIZE);
        return 0;
    }
    if (!boot_vector_is_valid(s_ulFwSize)){
        boot_set_error(BOOT_ERR_APP_INVALID);
        return 0;
    }

    if (boot_crc32((const uint8_t *)APP_START_ADDR, s_ulFwSize) != s_ulFwCrc){
        boot_set_error(BOOT_ERR_CRC_FAIL);
        return 0;
    }

    if (!boot_meta_write(s_ulFwSize, s_ulFwCrc)){
        boot_set_error(BOOT_ERR_FLASH);
        return 0;
    }

    s_bAppValid = 1;
    s_bCmdOk = 1;
    s_eErr = BOOT_ERR_NONE;
    return 1;
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

    /* 复位轮询 UART */
    usart_deinit(COM_PORT);

    /* 关闭所有 NVIC 中断使能并清 pending */
    for (i = 0U; i < 8U; i++){
        NVIC->ICER[i] = 0xFFFFFFFFUL;
        NVIC->ICPR[i] = 0xFFFFFFFFUL;
    }

    SysTick->CTRL = 0U;
    SCB->VTOR     = APP_START_ADDR;  /* 原来 0U, 改成 APP 基址 */

    /* 485 切回接收方向 */
    LED_485_ON;
}

bool boot_jump_to_app(void)
{
    uint32_t sp, pc;
	  volatile uint32_t i = 0;
    void   (*pReset)(void);

    if (!boot_app_verify()){
        s_bAppValid = 0;
        boot_set_error(BOOT_ERR_APP_INVALID);
        return 0;
    }

    sp = *(volatile uint32_t *)APP_START_ADDR;
    pc = *(volatile uint32_t *)(APP_START_ADDR + 4U);
    s_bAppValid = 1;
    
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

uint32_t boot_get_app_size(void)
{
    return s_ulFwSize;
}

uint32_t boot_get_app_crc(void)
{
    return s_ulFwCrc;
}

boot_err_t boot_get_error(void)
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
