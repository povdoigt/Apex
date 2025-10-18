#include "drivers/led.h"
#include "cmsis_gcc.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "main.h"
#include "stm32f4xx_hal_gpio.h"
#include <stdint.h>

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


void TASK_led(void *argument) {
    led_task_arg *arg = (led_task_arg *)argument;

    GPIO_TypeDef *port = arg->port;
    uint16_t pin = arg->pin;
    uint32_t first_delay = arg->first_delay;
    uint32_t period_off = arg->period_off;
    uint32_t period_on = arg->period_on;
    osSemaphoreId_t start_sem = arg->start_sem;

    bool is_on = false;

    osSemaphoreRelease(start_sem); // Release the semaphore to signal that the task is ready

    osDelay(first_delay); // Initial delay before starting the task

    for(;;) {
        HAL_GPIO_TogglePin(port, pin); // Toggle the LED state
        is_on = !is_on; // Toggle the state
        osDelay(is_on ? period_on : period_off); // Wait for the specified period
    }
}

void TASK_start_Led0R(void *argument) {

    const osSemaphoreAttr_t start_sem_attr = {
        .name = "TASK_start_Led0R"
    };

    osSemaphoreId_t start_sem = osSemaphoreNew(1, 0, &start_sem_attr);
    
    led_task_arg arg = {
        .port = LED0R_GPIO_Port,
        .pin = LED0R_Pin,
        .first_delay = 0,
        .period_off = 5000,
        .period_on = 1000,
        .start_sem = start_sem, // Semaphore for synchronization
    };

    const osThreadAttr_t TASK_led_attributes = {
        .name = "TASK_led",
        .stack_size = 128 * 4,
        .priority = (osPriority_t) osPriorityNormal,
    };

    osThreadNew(TASK_led, &arg, &TASK_led_attributes); // Create the LED task

    // Wait for the semaphore to start the LED task
    osSemaphoreAcquire(start_sem, osWaitForever);
    __NOP();

    // The LED task is now well initialized and can run independently
    // This function can return, and the LED task will continue running in the background.
    
    osThreadExit(); // Exit the thread, as the LED task is now running independently    
}

void TASK_start_Led0G(void *argument) {

    const osSemaphoreAttr_t start_sem_attr = {
        .name = "TASK_start_Led0G"
    };

    osSemaphoreId_t start_sem = osSemaphoreNew(1, 0, &start_sem_attr);
    
    led_task_arg arg = {
        .port = LED0G_GPIO_Port,
        .pin = LED0G_Pin,
        .first_delay = 0,
        .period_off = 1100,
        .period_on = 1000,
        .start_sem = start_sem, // Semaphore for synchronization
    };

    const osThreadAttr_t TASK_led_attributes = {
        .name = "TASK_led",
        .stack_size = 128 * 4,
        .priority = (osPriority_t) osPriorityNormal,
    };

    osThreadNew(TASK_led, &arg, &TASK_led_attributes); // Create the LED task

    // Wait for the semaphore to start the LED task
    osSemaphoreAcquire(start_sem, osWaitForever);
    __NOP();

    // The LED task is now well initialized and can run independently
    // This function can return, and the LED task will continue running in the background.
    
    osThreadExit(); // Exit the thread, as the LED task is now running independently    
}

// TASK_POOL_CREATE(ASYNC_LED_RGB_Blink);

// void ASYNC_LED_RGB_Blink_init(TASK *self, LED_RGB *led_rgb,
//                                uint8_t red, uint8_t green, uint8_t blue,
//                                uint32_t delay_on, uint32_t delay_off) {
//     ASYNC_LED_RGB_Blink_CONTEXT *context = (ASYNC_LED_RGB_Blink_CONTEXT*)self->context;

//     context->led_rgb = led_rgb;
//     context->red = red;
//     context->green = green;
//     context->blue = blue;
//     context->delay_on = delay_on;
//     context->delay_off = delay_off;

//     context->task_on = true; // Task is active
//     context->state = ASYNC_LED_RGB_Blink_ON;
//     context->next_time = HAL_GetTick() + delay_on;
// }

// TASK_RETURN ASYNC_LED_RGB_Blink(SCHEDULER *scheduler, TASK *self) {
//     ASYNC_LED_RGB_Blink_CONTEXT *context = (ASYNC_LED_RGB_Blink_CONTEXT*)self->context;

//     uint32_t current_tick = HAL_GetTick();
//     switch (context->state) {
//     case  ASYNC_LED_RGB_Blink_ON: {
//         if (!context->task_on) {
//             LED_RGB_SetColor(context->led_rgb, 0, 0, 0);
//             return TASK_RETURN_STOP; // Stop the task if task_on is false
//         }

//         if (current_tick >= context->next_time) {
//             LED_RGB_SetColor(context->led_rgb, 0, 0, 0);
//             context->state = ASYNC_LED_RGB_Blink_OFF;
//         }
//         break; }
//     case ASYNC_LED_RGB_Blink_OFF: {
//         if (current_tick >= context->next_time) {
//             LED_RGB_SetColor(context->led_rgb, context->red, context->green, context->blue);
//             context->state = ASYNC_LED_RGB_Blink_OFF;
//         }
//         break; }
//     }

//     return TASK_RETURN_IDLE;
// }