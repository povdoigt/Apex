#include "project.h"
#include "cmsis_os2.h"
#include "main.h"
#include "scheduler.h"
#include <stdint.h>



// Allocate memory pool for TASK_led (typically in a .c file)
TASK_POOL_ALLOCATE(TASK_led_test);
void TASK_led_test(void *argument) {
    TASK_led_test_ARGS *args = (TASK_led_test_ARGS*) argument;
    while (1) {
        HAL_GPIO_TogglePin(args->led->port, args->led->pin);
        osDelay(args->delay);
    }
}

led_t led0r = { .port=LED0R_GPIO_Port, .pin=LED0R_Pin};
led_t led0g = { .port=LED0G_GPIO_Port, .pin=LED0G_Pin};
led_t led0b = { .port=LED0B_GPIO_Port, .pin=LED0B_Pin};

TASK_led_test_ARGS task_led_args_0r = { .led = &led0r, .delay = 2000};
TASK_led_test_ARGS task_led_args_0g = { .led = &led0g, .delay = 1000};
TASK_led_test_ARGS task_led_args_0b = { .led = &led0b, .delay = 500};


void setup(void) {

    // Call the kernel to create the memory pool for TASK_led
    // Should be called just between osKernelInitialize() and osKernelStart() in the main
    // function so that the pool is created before any task tries to allocate from it,
    // and after the kernel is initialized.
    TASK_POOL_CREATE(TASK_led_test);

    osThreadAttr_t task_attr = { .priority = (osPriority_t) osPriorityNormal };

    task_attr.name = "TASK_led_test_0r";
    OS_THREAD_NEW_CSTM(TASK_led_test, task_led_args_0r, task_attr, osWaitForever);

    task_attr.name = "TASK_led_test_0g";
    OS_THREAD_NEW_CSTM(TASK_led_test, task_led_args_0g, task_attr, osWaitForever);

    task_attr.name = "TASK_led_test_0b";
    OS_THREAD_NEW_CSTM(TASK_led_test, task_led_args_0b, task_attr, osWaitForever);
}


void loop(void) {
    // No use, RTOS scheduler
}