#include "utils/tools.h"

#include "peripherals/tim.h"

#include <stdio.h>

void float_format(char *buff, float num, int precision, int width) {
    float num_abs = num < 0 ? -num : num;

    long int num_up = (int)num_abs;
    long int num_dw = (int)((num_abs - num_up) * pow(10, precision));

    int num_up_width = width - precision - 2; // 1 for the dot and 1 for the sign
    long int max_up = pow(10, num_up_width) - 1;

    num_up = num_up > max_up ? max_up : num_up;

    if (num < 0) {
        sprintf(buff, "-%lu.%lu", num_up, num_dw);
    } else {
        sprintf(buff, "+%lu.%lu", num_up, num_dw);
    }
}


int count_nbr_elems(char buffer[], char sep) {
    int i = 1;
    for (int j = 0; buffer[j]; ++j) {
        if (buffer[j] == sep)
            ++i;
    }
    return i;
}

void set_elems_from_csv(char **elems, char buffer[], char sep, int nbr_elems) {
    int i = 0;
    int count = 0;

    buffer[strlen(buffer) - 1] = '\0';
    while (buffer[i] != '\0' && count < nbr_elems) {
        elems[count] = buffer + i;
        ++count;

        while (buffer[i] != sep && buffer[i] != '\0')
            ++i;

        if (buffer[i] == sep) {
            buffer[i] = '\0';
        }
        ++i;
    }
} 

void set_line_to_csv(char **elems, char buffer[], char sep, int nbr_elems) {
    char str_sep[2] = {sep, '\0'}; 
    buffer[0] = '\0';
    for (int i = 0; i < nbr_elems; ++i) {
        strcat(buffer, elems[i]);
        if (i < nbr_elems - 1)
            strcat(buffer, str_sep);
    }
    strcat(buffer, "\n");
}

HAL_StatusTypeDef TIM_Delay_Micro(uint32_t delay) {
    HAL_StatusTypeDef status;
    status = HAL_TIM_Base_Start(&htim9);
    if (status != HAL_OK) { return status; }
    __HAL_TIM_SET_COUNTER(&htim9, 0);
    while (__HAL_TIM_GET_COUNTER(&htim9) < delay);
    status = HAL_TIM_Base_Stop(&htim9);
    return status;  
}

// void SPI_HandleTypeDef_flag_init(SPI_HandleTypeDef_flag *hspi_flag, SPI_HandleTypeDef* hspi) {
//     hspi_flag->hspi = hspi;
//     hspi_flag->is_used = false;
// }


// SysTick does not decrement on microseconds, due to lack of prescaler and limited divider.
// Scale is adjusted in the return formula.
// The unmodified example for this code can be found on:
// https://electronics.stackexchange.com/questions/521020/stm32-create-a-microsecond-timer
// uint32_t GetMicros(void)
// {
//     uint32_t ms;
//     uint32_t st;
//     uint32_t range;

//     // do  // Retreive SysTick->VAL, retry on a ms transition because that will reload SysTick
//     // {
//         ms = HAL_GetTick();
//         st = SysTick->VAL;
//     //     asm volatile("nop");
//     //     asm volatile("nop");
//     // } while (ms != HAL_GetTick());

//     range = (SysTick->LOAD + 1);
//     return (ms * 1000) + ((range - st) / (range / 1000));
// }


// TASK_POOL_CREATE(ASYNC_Delay);

// void ASYNC_Delay_ms_init(TASK *self, uint32_t delay) {
//     ASYNC_Delay_CONTEXT *context = (ASYNC_Delay_CONTEXT*)self->context;
//     context->stop_time = HAL_GetTick() + delay;
// }

// TASK_RETURN ASYNC_Delay_ms(SCHEDULER *scheduler, TASK *self) {
//     ASYNC_Delay_CONTEXT *context = (ASYNC_Delay_CONTEXT*)self->context;
//     UNUSED(scheduler);

//     // uint32_t current_tick = HAL_GetTick();

//     if (HAL_GetTick() >= context->stop_time) {
//         return TASK_RETURN_STOP;
//     }
//     return TASK_RETURN_IDLE;
// }

// void ASYNC_Delay_us_init(TASK *self, uint32_t delay) {
//     ASYNC_Delay_CONTEXT *context = (ASYNC_Delay_CONTEXT*)self->context;
//     context->stop_time = GetMicros() + delay;
// }

// TASK_RETURN ASYNC_Delay_us(SCHEDULER *scheduler, TASK *self) {
//     ASYNC_Delay_CONTEXT *context = (ASYNC_Delay_CONTEXT*)self->context;
//     UNUSED(scheduler);

//     // uint32_t current_tick = GetMicros();

//     if (GetMicros() >= context->stop_time) {
//         return TASK_RETURN_STOP;
//     }
//     return TASK_RETURN_IDLE;
// }




