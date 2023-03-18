#include "main.h"

#include <application/controller.h>
#include <application/generator.h>
#include <application/timer_handler.h>
#include <application/visualizer.h>
#include <stdlib.h>
#include <string.h>

#define MAX_RX_SIZE 15

static void module_start(void);
static void module_stop(void);

static uint32_t visualizer_timer;
static bool controller_status = false;

void controller_init(void) {
    module_stop();
    visualizer_init();
    timer_restart(&visualizer_timer);
}

void controller_handler(void) {
    if (controller_status && timer_wait_ms(visualizer_timer, visualizer_get_period())) {
        timer_restart(&visualizer_timer);
        visualizer_print_channels();
    }
}

void controller_receive_message(char* message, uint32_t size) {
    if (size > MAX_RX_SIZE) {
        return;
    }

    if (strncmp(message, "start", 5) == 0) {
        module_start();
    } else if (strncmp(message, "stop", 4) == 0) {
        module_stop();
    } else if (strncmp(message, "add", 3) == 0) {
        visualizer_update_channels(atoi(&message[3]), true);
    } else if (strncmp(message, "remove", 6) == 0) {
        visualizer_update_channels(atoi(&message[6]), false);
    } else if (strncmp(message, "freq", 4) == 0) {
        visualizer_update_frequency(atoi(&message[4]));
    } else if (strncmp(message, "sine_freq", 9) == 0) {
        generator_update_sine_frequency(atoi(&message[9]));
    } else if (strncmp(message, "pwm_freq", 8) == 0) {
        generator_update_pwm_frequency(atoi(&message[8]));
    } else if (strncmp(message, "pwm_duty", 8) == 0) {
        generator_update_pwm_duty(atoi(&message[8]));
    }
}

static void module_start(void) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
    generator_start();
    controller_status = true;
}

static void module_stop(void) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
    generator_stop();
    controller_status = false;
}
