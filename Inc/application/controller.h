#pragma once

#include <stdint.h>

void ControllerInit(void);
void ControllerHandler(void);
void controller_receive_message(char* message, uint32_t size);