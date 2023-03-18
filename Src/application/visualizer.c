#include "stm32f1xx_hal.h"
#include "usbd_cdc_if.h"

#include <application/visualizer.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_CHANNEL   7
#define MAX_FREQUENCY 200
#define MIN_FREQUENCY 1
#define MAX_TX_SIZE   100
#define GAIN_ADC      (4.095 / 3.3)

static uint16_t configured_period_ms   = 1000;
static uint8_t activated_channels_flag = 1;
static uint16_t adc_buf[MAX_CHANNEL + 1];

extern ADC_HandleTypeDef hadc1;

void visualizer_init(void) {
    HAL_ADCEx_Calibration_Start(&hadc1);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, MAX_CHANNEL + 1);
}

void visualizer_update_frequency(int32_t value) {
    char stringToSend[MAX_TX_SIZE];
    int32_t tam;

    if (value >= MIN_FREQUENCY && value <= MAX_FREQUENCY) {
        configured_period_ms = 1000 / value;

        tam = sprintf(stringToSend, "Frequency set as %d Hz, period is %d ms.\n",
                      1000 / configured_period_ms, configured_period_ms);

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

void visualizer_print_channels(void) {
    if (!activated_channels_flag) {
        return;
    }
    char stringToSend[MAX_TX_SIZE];
    uint8_t index = 0;

    for (uint8_t i = 0; i <= MAX_CHANNEL; ++i) {
        if (activated_channels_flag & (1 << i)) {
            index +=
                sprintf(stringToSend + index, "%.1f\t", (float)(adc_buf[i] / GAIN_ADC));
        }
    }
    sprintf(stringToSend + index++, "\n");

    if (index > MAX_TX_SIZE) {
        return;
    }
    CDC_Transmit_FS((uint8_t*)stringToSend, index);
}

uint16_t visualizer_get_period(void) {
    return configured_period_ms;
}

void visualizer_update_channels(uint8_t channel, bool status) {
    char stringToSend[MAX_TX_SIZE];
    int32_t index = 0;

    if (channel > MAX_CHANNEL) {
        index += sprintf(stringToSend, "Channel not allowed. ");
    } else {
        if (status) {
            activated_channels_flag |= (1 << channel);
            index += sprintf(stringToSend + index, "Channel %d added. ", channel);
        } else {
            activated_channels_flag &= ~(1 << channel);
            index += sprintf(stringToSend + index, "Channel %d removed. ", channel);
        }
    }
    index += sprintf(stringToSend + index, "Added channels:");
    for (uint8_t i = 0; i <= MAX_CHANNEL; ++i) {
        if (activated_channels_flag & (1 << i)) {
            index += sprintf(stringToSend + index, " %d", i);
        }
    }
    sprintf(stringToSend + index++, "\n");

    CDC_Transmit_FS((uint8_t*)stringToSend, index);
}
