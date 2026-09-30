#ifndef BOOT_PROTOCOL_H
#define BOOT_PROTOCOL_H

#include "gd32f10x.h"

void boot_protocol_init(uint32_t baudrate);
void boot_protocol_poll(void);

#endif