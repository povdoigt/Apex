#include "drivers/led.h"

void LED_Init(LED *led, TIM_HandleTypeDef *timer, uint8_t channel) {
    led->timer = timer;
    led->channel = channel;

    TIM_set_frequency(timer, LED_TIMER_FREQUENCY); // Set the timer frequency for PWM
    HAL_TIM_PWM_Start(timer, channel);
}

void LED_SetBrightness(LED *led, uint8_t brightness) {
    // from [0 255] to [0 tim->Period]
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(led->timer);
    uint32_t brightness_32 = ((float)brightness / 255.0f) * period;
    __HAL_TIM_SET_COMPARE(led->timer, led->channel, period - brightness_32);
}


void LED_RGB_Init(LED_RGB *led_rgb, TIM_HandleTypeDef *timer,
                   uint8_t red_channel,
                   uint8_t green_channel,
                   uint8_t blue_channel) {
    LED_Init(&led_rgb->red, timer, red_channel);
    LED_Init(&led_rgb->green, timer, green_channel);
    LED_Init(&led_rgb->blue, timer, blue_channel);
    LED_RGB_SetColor(led_rgb, 0, 0, 0); // Initialize RGB LED to off
}

void LED_RGB_SetColor(LED_RGB *led_rgb, uint8_t red, uint8_t green, uint8_t blue) {
    LED_SetBrightness(&led_rgb->red, red);
    LED_SetBrightness(&led_rgb->green, green);
    LED_SetBrightness(&led_rgb->blue, blue);
}


TASK_POOL_CREATE(ASYNC_LED_RGB_Blink);

void ASYNC_LED_RGB_Blink_init(TASK *self, LED_RGB *led_rgb,
                               uint8_t red, uint8_t green, uint8_t blue,
                               uint32_t delay_on, uint32_t delay_off) {
    ASYNC_LED_RGB_Blink_CONTEXT *context = (ASYNC_LED_RGB_Blink_CONTEXT*)self->context;

    context->led_rgb = led_rgb;
    context->red = red;
    context->green = green;
    context->blue = blue;
    context->delay_on = delay_on;
    context->delay_off = delay_off;

    context->task_on = true; // Task is active
    context->state = ASYNC_LED_RGB_Blink_ON;
    context->next_time = HAL_GetTick() + delay_on;
}

TASK_RETURN ASYNC_LED_RGB_Blink(SCHEDULER *scheduler, TASK *self) {
    ASYNC_LED_RGB_Blink_CONTEXT *context = (ASYNC_LED_RGB_Blink_CONTEXT*)self->context;

    uint32_t current_tick = HAL_GetTick();
    switch (context->state) {
    case  ASYNC_LED_RGB_Blink_ON: {
        if (!context->task_on) {
            LED_RGB_SetColor(context->led_rgb, 0, 0, 0);
            return TASK_RETURN_STOP; // Stop the task if task_on is false
        }

        if (current_tick >= context->next_time) {
            LED_RGB_SetColor(context->led_rgb, 0, 0, 0);
            context->state = ASYNC_LED_RGB_Blink_OFF;
        }
        break; }
    case ASYNC_LED_RGB_Blink_OFF: {
        if (current_tick >= context->next_time) {
            LED_RGB_SetColor(context->led_rgb, context->red, context->green, context->blue);
            context->state = ASYNC_LED_RGB_Blink_OFF;
        }
        break; }
    }

    return TASK_RETURN_IDLE;
}