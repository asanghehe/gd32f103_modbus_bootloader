#include "boot_protocol.h"
#include "bootloader.h"
#include "bsp_usart.h"
#include "bsp_led.h"
#include "gd32f10x_libopt.h"

#define FRAME_SOF_0          0xA5U
#define FRAME_SOF_1          0x5AU
#define FRAME_MAX_DATA       260U
#define FRAME_HEADER_SIZE    6U
#define FRAME_CRC_SIZE       2U
#define FRAME_MAX_SIZE       (FRAME_HEADER_SIZE + FRAME_MAX_DATA + FRAME_CRC_SIZE)
#define FRAME_RESPONSE       0x80U

#define CMD_INFO             0x01U
#define CMD_BEGIN            0x10U
#define CMD_DATA             0x11U
#define CMD_VERIFY           0x12U
#define CMD_JUMP             0x13U
#define CMD_RESET            0x14U
#define CMD_READ             0x15U

static uint8_t s_rxFrame[FRAME_MAX_SIZE];
static uint16_t s_rxCount;
static uint16_t s_expectedSize;

static uint16_t protocol_crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t index;
    uint8_t bit;

    for (index = 0U; index < length; index++){
        crc ^= (uint16_t)data[index] << 8;
        for (bit = 0U; bit < 8U; bit++){
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static uint32_t read_u32_le(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static void write_u16_le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
}

static void write_u32_le(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
    data[2] = (uint8_t)(value >> 16);
    data[3] = (uint8_t)(value >> 24);
}

static void uart_send(const uint8_t *data, uint16_t length)
{
    uint16_t index;

    LED_485_OFF;
    for (index = 0U; index < length; index++){
        while (RESET == usart_flag_get(COM_PORT, USART_FLAG_TBE)){
        }
        usart_data_transmit(COM_PORT, data[index]);
    }
    while (RESET == usart_flag_get(COM_PORT, USART_FLAG_TC)){
    }
    LED_485_ON;
}

static void send_frame(uint8_t command, uint8_t sequence, const uint8_t *payload, uint16_t length)
{
    uint8_t frame[FRAME_MAX_SIZE];
    uint16_t crc;
    uint16_t index;

    if (length > FRAME_MAX_DATA){
        return;
    }
    frame[0] = FRAME_SOF_0;
    frame[1] = FRAME_SOF_1;
    frame[2] = command;
    frame[3] = sequence;
    write_u16_le(&frame[4], length);
    for (index = 0U; index < length; index++){
        frame[FRAME_HEADER_SIZE + index] = payload[index];
    }
    crc = protocol_crc16(&frame[2], (uint16_t)(4U + length));
    write_u16_le(&frame[FRAME_HEADER_SIZE + length], crc);
    uart_send(frame, (uint16_t)(FRAME_HEADER_SIZE + length + FRAME_CRC_SIZE));
}

static void send_result(uint8_t command, uint8_t sequence, bool success)
{
    uint8_t response[3];

    response[0] = success ? 0U : 1U;
    write_u16_le(&response[1], (uint16_t)boot_get_error());
    send_frame((uint8_t)(command | FRAME_RESPONSE), sequence, response, sizeof(response));
}

static void send_info(uint8_t sequence)
{
    uint8_t info[22];

    write_u16_le(&info[0], BOOT_VERSION);
    write_u16_le(&info[2], boot_get_status());
    write_u16_le(&info[4], (uint16_t)boot_get_error());
    write_u32_le(&info[6], boot_get_app_size());
    write_u32_le(&info[10], boot_get_app_crc());
    write_u32_le(&info[14], APP_START_ADDR);
    write_u32_le(&info[18], APP_MAX_SIZE);
    send_frame((uint8_t)(CMD_INFO | FRAME_RESPONSE), sequence, info, sizeof(info));
}

static void handle_frame(void)
{
    uint8_t command = s_rxFrame[2];
    uint8_t sequence = s_rxFrame[3];
    uint16_t length = (uint16_t)s_rxFrame[4] | ((uint16_t)s_rxFrame[5] << 8);
    const uint8_t *payload = &s_rxFrame[FRAME_HEADER_SIZE];
    bool success = 0;

    switch (command){
    case CMD_INFO:
        if (length == 0U){
            send_info(sequence);
        }
        break;
    case CMD_BEGIN:
        if (length == 8U){
            success = boot_cmd_begin_update(read_u32_le(payload), read_u32_le(&payload[4]));
            send_result(command, sequence, success);
        }
        break;
    case CMD_DATA:
        if ((length >= 5U) && (length <= FRAME_MAX_DATA)){
            success = boot_cmd_write(read_u32_le(payload), &payload[4], (uint16_t)(length - 4U));
            send_result(command, sequence, success);
        }
        break;
    case CMD_VERIFY:
        if (length == 0U){
            success = boot_cmd_check_crc();
            send_result(command, sequence, success);
        }
        break;
    case CMD_JUMP:
        if (length == 0U){
            success = boot_app_is_valid();
            if (!success){
                boot_set_error(BOOT_ERR_APP_INVALID);
            }
            send_result(command, sequence, success);
            if (success){
                (void)boot_jump_to_app();
            }
        }
        break;
    case CMD_RESET:
        if (length == 0U){
            boot_set_error(BOOT_ERR_NONE);
            send_result(command, sequence, 1);
            boot_system_reset();
        }
        break;
    case CMD_READ:
        if ((length == 6U) && (read_u32_le(payload) <= APP_MAX_SIZE) &&
            (((uint16_t)payload[4] | ((uint16_t)payload[5] << 8)) != 0U) &&
            (((uint16_t)payload[4] | ((uint16_t)payload[5] << 8)) <= 256U) &&
            ((uint32_t)read_u32_le(payload) + payload[4] + ((uint16_t)payload[5] << 8) <= APP_MAX_SIZE)){
            uint8_t response[FRAME_MAX_DATA];
            uint32_t read_offset = read_u32_le(payload);
            uint16_t read_length = (uint16_t)payload[4] | ((uint16_t)payload[5] << 8);
            uint16_t index;
            response[0] = 0U;
            write_u16_le(&response[1], (uint16_t)boot_get_error());
            for (index = 0U; index < read_length; index++){
                response[3U + index] = *(const uint8_t *)(APP_START_ADDR + read_offset + index);
            }
            send_frame((uint8_t)(command | FRAME_RESPONSE), sequence, response, (uint16_t)(3U + read_length));
        }else{
            boot_set_error(BOOT_ERR_BAD_ADDR);
            send_result(command, sequence, 0);
        }
        break;
    default:
        boot_set_error(BOOT_ERR_BAD_ADDR);
        send_result(command, sequence, 0);
        break;
    }
}

static void receive_byte(uint8_t byte)
{
    if (s_rxCount == 0U){
        if (byte == FRAME_SOF_0){
            s_rxFrame[s_rxCount++] = byte;
        }
        return;
    }
    if (s_rxCount == 1U){
        if (byte == FRAME_SOF_1){
            s_rxFrame[s_rxCount++] = byte;
        }else if (byte != FRAME_SOF_0){
            s_rxCount = 0U;
        }
        return;
    }

    s_rxFrame[s_rxCount++] = byte;
    if (s_rxCount == FRAME_HEADER_SIZE){
        uint16_t payload_length = (uint16_t)s_rxFrame[4] | ((uint16_t)s_rxFrame[5] << 8);
        if (payload_length > FRAME_MAX_DATA){
            s_rxCount = 0U;
            return;
        }
        s_expectedSize = (uint16_t)(FRAME_HEADER_SIZE + payload_length + FRAME_CRC_SIZE);
    }
    if ((s_expectedSize != 0U) && (s_rxCount == s_expectedSize)){
        uint16_t payload_length = (uint16_t)s_rxFrame[4] | ((uint16_t)s_rxFrame[5] << 8);
        uint16_t expected_crc = (uint16_t)s_rxFrame[FRAME_HEADER_SIZE + payload_length] |
            ((uint16_t)s_rxFrame[FRAME_HEADER_SIZE + payload_length + 1U] << 8);
        if (protocol_crc16(&s_rxFrame[2], (uint16_t)(4U + payload_length)) == expected_crc){
            handle_frame();
        }
        s_rxCount = 0U;
        s_expectedSize = 0U;
    }
}

void boot_protocol_init(uint32_t baudrate)
{
    rcu_periph_clock_enable(COM_PORT_RCU);
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOD);

    gpio_init(COM_PORT_T, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, COM_PORT_PIN_TX);
    gpio_init(COM_PORT_R, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, COM_PORT_PIN_RX);
    LED_485_ON;

    usart_deinit(COM_PORT);
    usart_baudrate_set(COM_PORT, baudrate);
    usart_word_length_set(COM_PORT, USART_WL_8BIT);
    usart_stop_bit_set(COM_PORT, USART_STB_1BIT);
    usart_parity_config(COM_PORT, USART_PM_NONE);
    usart_hardware_flow_rts_config(COM_PORT, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(COM_PORT, USART_CTS_DISABLE);
    usart_receive_config(COM_PORT, USART_RECEIVE_ENABLE);
    usart_transmit_config(COM_PORT, USART_TRANSMIT_ENABLE);
    usart_enable(COM_PORT);

    s_rxCount = 0U;
    s_expectedSize = 0U;
}

void boot_protocol_poll(void)
{
    while (RESET != usart_flag_get(COM_PORT, USART_FLAG_RBNE)){
        receive_byte((uint8_t)usart_data_receive(COM_PORT));
    }
}