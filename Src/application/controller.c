
#include <application/controller.h>
#include <application/timer_handler.h>
#include <application/visualizer.h>
#include "main.h"
#include "stm32f1xx_hal.h"

#include <string.h>

#define MAX_RX_SIZE 15

extern TIM_HandleTypeDef htim3;

void module_start();
void module_stop();
void active_ch(uint16_t chanel_atv);
void change_freq();
void visualizer_update_frequency(int32_t requestedFrequency);

uint16_t adc_atv = 0, pwm_v = 0;
uint32_t tim3_prs = 71, tim2_prs = 71;
extern uint16_t sen_pwm[100];
extern uint8_t sen_act;

uint16_t i;
double ang, sin_tmp;
char char_aux;

static uint32_t visualizer_timer;
static bool controller_status = false;

void ControllerInit(void) {

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    module_stop();
    visualizer_init();
    timer_restart(&visualizer_timer);
}

void ControllerHandler(void) {

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
    } else if (strncmp(message, "pwm_freq", 8) == 0) {
        change_freq();
    } else if (strncmp(message, "pwm_v", 5) == 0) {
        pwm_v = (message[5] - 48) * 1000 + (message[6] - 48) * 100
                + (message[7] - 48) * 10 + message[8] - 48;
        if (pwm_v > 1000)
            pwm_v = 1000;
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm_v);
    }
}

void module_start() {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
    HAL_TIM_Base_Start_IT(&htim3);
    HAL_TIM_Base_Start(&htim3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    controller_status = true;
}

void module_stop() {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    HAL_TIM_Base_Stop_IT(&htim3);
    controller_status = false;
}

void active_ch(uint16_t chanel_atv) {
    module_stop();
    adc_atv = chanel_atv;
}

void change_freq() {
    module_stop();
    // if (message[4] == '1' && message[5] == '0' && message[6] == '0') {
    //     sen_act  = 1;
    //     tim3_prs = 7;
    //     MX_TIM3_Init();
    // } else if (message[4] == '1' && message[5] == '0') {
    //     sen_act  = 1;
    //     tim3_prs = 71;
    //     MX_TIM3_Init();
    // } else if (message[4] == '1') {
    //     sen_act  = 1;
    //     tim3_prs = 719;
    //     MX_TIM3_Init();
    // } else if (message[4] == '5' && message[5] == '0') {
    //     sen_act  = 1;
    //     tim3_prs = 13;
    //     MX_TIM3_Init();
    // } else if (message[4] == '5') {
    //     sen_act  = 1;
    //     tim3_prs = 143;
    //     MX_TIM3_Init();
    // } else if (message[4] == '3' && message[5] == '0') {
    //     sen_act  = 1;
    //     tim3_prs = 23;
    //     MX_TIM3_Init();
    // } else if (message[4] == '2' && message[5] == '0') {
    //     sen_act  = 1;
    //     tim3_prs = 35;
    //     MX_TIM3_Init();
    // } else if (message[4] == '2') {
    //     sen_act  = 1;
    //     tim3_prs = 359;
    //     MX_TIM3_Init();
    // } else if (message[4] == '0' && message[5] == '5') {
    //     sen_act  = 1;
    //     tim3_prs = 1439;
    //     MX_TIM3_Init();
    // } else if (message[4] == '0' && message[5] == '2') {
    //     sen_act  = 1;
    //     tim3_prs = 3599;
    //     HAL_TIM_PWM_Start();
    //     MX_TIM3_Init();
    // } else if (message[4] == '0') {
    //     sen_act = 0;
    //     __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm_v);
    // }
}
