#ifndef UART_MUX_H
#define UART_MUX_H

#include "stm32f4xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum uart_mux_channel_t {
    UART_MUX_CHANNEL_0 = 0,
    UART_MUX_CHANNEL_1 = 1
} uart_mux_channel_t;

typedef struct uart_mux_t {
    uart_mux_channel_t   channel;

    GPIO_TypeDef        *gpio_port;
    uint16_t             gpio_pin;
} uart_mux_t;

void uart_mux_init(uart_mux_t *mux, GPIO_TypeDef *gpio_port, uint16_t gpio_pin);
void uart_mux_set(uart_mux_t *mux, uart_mux_channel_t channel);
uart_mux_channel_t uart_mux_get_channel(uart_mux_t *mux);

#endif /* UART_MUX_H */
