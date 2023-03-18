#pragma once

#include <stdint.h>

void controller_init(void);
void controller_handler(void);
void controller_receive_message(char* message, uint32_t size);