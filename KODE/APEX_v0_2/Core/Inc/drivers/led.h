#ifndef LED_H
#define LED_H


#include "stm32f4xx_hal.h"
#include "peripherals/tim.h"
#include "utils/scheduler.h"

#define LED_TIMER_FREQUENCY 10000 // Frequency for PWM (10 kHz)

typedef struct LED {
    TIM_HandleTypeDef *timer; // Timer handle for PWM
    uint8_t channel; // PWM channel
} LED;

void LED_Init(LED *led, TIM_HandleTypeDef *timer, uint8_t channel);
void LED_SetBrightness(LED *led, uint8_t brightness);

typedef struct LED_RGB {
    LED red;
    LED green;
    LED blue;
} LED_RGB;

void LED_RGB_Init(LED_RGB *led_rgb, TIM_HandleTypeDef *timer,
                  uint8_t red_channel,
                  uint8_t green_channel,
                  uint8_t blue_channel);

void LED_RGB_SetColor(LED_RGB *led_rgb, uint8_t red, uint8_t green, uint8_t blue);


#define ASYNC_LED_RGB_Blink_NUMBER 5

typedef enum ASYNC_LED_RGB_Blink_STATE {
    ASYNC_LED_RGB_Blink_ON,
    ASYNC_LED_RGB_Blink_OFF,
} ASYNC_LED_RGB_Blink_STATE;

typedef struct ASYNC_LED_RGB_Blink_CONTEXT {
    LED_RGB *led_rgb;
    uint8_t red;
    uint8_t green;
    uint8_t blue;
    uint32_t delay_on;
    uint32_t delay_off;

    bool task_on;
    ASYNC_LED_RGB_Blink_STATE state;
    uint32_t next_time; // Next time to toggle the LED
} ASYNC_LED_RGB_Blink_CONTEXT;

extern TASK_POOL_CREATE(ASYNC_LED_RGB_Blink);

void ASYNC_LED_RGB_Blink_init(TASK *self, LED_RGB *led_rgb,
                              uint8_t red, uint8_t green, uint8_t blue,
                              uint32_t delay_on, uint32_t delay_off);
TASK_RETURN ASYNC_LED_RGB_Blink(SCHEDULER *scheduler, TASK *self);


#define ASYNC_LED_RGB_Step_NUMBER 5

typedef enum ASYNC_LED_RGB_Step_STATE {
    ASYNC_LED_RGB_Step_START,
    ASYNC_LED_RGB_Step_WAIT_ON_UP,
    ASYNC_LED_RGB_Step_WAIT_ON_DOWN,
    ASYNC_LED_RGB_Step_WAIT_OFF,
    ASYNC_LED_RGB_Step_END
} ASYNC_LED_RGB_Step_STATE;

typedef struct ASYNC_LED_RGB_Step_CONTEXT {
    LED_RGB *led_rgb;
    uint8_t red;
    uint8_t green;
    uint8_t blue;
    uint32_t delay_on;
    uint32_t delay_off;

    bool task_on;
    ASYNC_LED_RGB_Step_STATE state;
    size_t step; // Current step in the sequence
    uint32_t next_time; // Next time to toggle the LED
} ASYNC_LED_RGB_Step_CONTEXT;

extern TASK_POOL_CREATE(ASYNC_LED_RGB_Step);

void ASYNC_LED_RGB_Step_init(TASK *self, LED_RGB *led_rgb,
                             uint8_t red, uint8_t green, uint8_t blue,
                             uint32_t delay_on, uint32_t delay_off);
TASK_RETURN ASYNC_LED_RGB_Step(SCHEDULER *scheduler, TASK *self);


#endif /* LED_H */