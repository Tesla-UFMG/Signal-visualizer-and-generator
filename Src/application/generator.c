
#include "stm32f1xx_hal.h"
#include "usbd_cdc_if.h"

#include <application/generator.h>

#define MAX_TX_SIZE        100
#define MAX_FREQUENCY      10000
#define MIN_FREQUENCY      2
#define MAX_DUTY           100
#define MIN_DUTY           0
#define PWM_PRESCALER_GAIN 100000

void set_duty();

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;

static uint16_t configured_period = 9999;
static uint16_t configured_duty   = 50;

void generator_start(void) {
    HAL_TIM_Base_Start(&htim4);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
    set_duty();
}

void generator_stop(void) {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
    HAL_TIM_Base_Stop(&htim4);
}

void generator_update_pwm_frequency(int32_t value) {
    char stringToSend[MAX_TX_SIZE];
    int32_t tam;

    if (value >= MIN_FREQUENCY && value <= MAX_FREQUENCY) {
        configured_period = (PWM_PRESCALER_GAIN / value) - 1;

        tam = sprintf(stringToSend, "Frequency set as %d Hz, period is %d.\n",
                      PWM_PRESCALER_GAIN / (configured_period + 1), configured_period);
        htim4.Init.Period = configured_period;
        HAL_TIM_Base_Init(&htim4);
        set_duty();

    } else {
        tam = sprintf(stringToSend,
                      "Value not allowed, allowed frequencies are %d to %d Hz.\n",
                      MIN_FREQUENCY, MAX_FREQUENCY);
    }

    if (tam > MAX_TX_SIZE) {
        return;
    }
    CDC_Transmit_FS((uint8_t*)stringToSend, tam);
}
void generator_update_pwm_duty(int32_t value) {
    char stringToSend[MAX_TX_SIZE];
    int32_t tam;
    if (value >= MIN_DUTY && value <= MAX_DUTY) {

        configured_duty = value;

        set_duty();

        tam = sprintf(stringToSend, "Duty cycle set as %d%%, compare value is %d.\n",
                      configured_duty, (configured_duty * configured_period) / 100);

    } else {
        tam = sprintf(stringToSend,
                      "Value not allowed, enter a duty cycle value of %d to %d%%. \n",
                      MIN_DUTY, MAX_DUTY);
    }

    if (tam > MAX_TX_SIZE) {
        return;
    }
    CDC_Transmit_FS((uint8_t*)stringToSend, tam);
}

void set_duty() {
    const uint16_t compare_value = (configured_duty * configured_period) / 100;
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, compare_value);
}

void generator_update_sine_frequency(int32_t value) {}
