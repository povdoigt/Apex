#include "uart_mux.h"

void uart_mux_init(uart_mux_t *mux, GPIO_TypeDef *gpio_port, uint16_t gpio_pin) {
    mux->channel = UART_MUX_CHANNEL_0; // Default to channel 0
    mux->gpio_port = gpio_port;
    mux->gpio_pin = gpio_pin;

    // Initialize the GPIO pin (assuming it's already configured as output)
    HAL_GPIO_WritePin(mux->gpio_port, mux->gpio_pin, GPIO_PIN_RESET);
}

void uart_mux_set(uart_mux_t *mux, uart_mux_channel_t channel) {
    mux->channel = channel;
    HAL_GPIO_WritePin(mux->gpio_port, mux->gpio_pin, channel == UART_MUX_CHANNEL_1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

uart_mux_channel_t uart_mux_get_channel(uart_mux_t *mux) {
    return mux->channel;
}