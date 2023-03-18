#pragma once

#include <stdint.h>

void generator_start(void);
void generator_stop(void);
void generator_update_sine_frequency(int32_t value);
void generator_update_pwm_frequency(int32_t value);
void generator_update_pwm_duty(int32_t value);
