#ifndef UART_MUX_H
#define UART_MUX_H

#include "stm32f4xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct uart_mux_t {
    bool             state;     // Current state of the mux (true for 2nd UART, false for 1st UART)

    GPIO_TypeDef    *gpio_port;
    uint16_t         gpio_pin;  
} uart_mux_t;

void uart_mux_init(uart_mux_t *mux, GPIO_TypeDef *gpio_port, uint16_t gpio_pin);
void uart_mux_set(uart_mux_t *mux, bool state);
bool uart_mux_get_state(uart_mux_t *mux);

#endif /* UART_MUX_H */
