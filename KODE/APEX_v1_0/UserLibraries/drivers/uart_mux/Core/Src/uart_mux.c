#include "uart_mux.h"

void uart_mux_init(uart_mux_t *mux, GPIO_TypeDef *gpio_port, uint16_t gpio_pin) {
    mux->state = false; // Default to 1st UART
    mux->gpio_port = gpio_port;
    mux->gpio_pin = gpio_pin;

    // Initialize the GPIO pin (assuming it's already configured as output)
    HAL_GPIO_WritePin(mux->gpio_port, mux->gpio_pin, GPIO_PIN_RESET);
}

void uart_mux_set(uart_mux_t *mux, bool state) {
    mux->state = state;
    HAL_GPIO_WritePin(mux->gpio_port, mux->gpio_pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

bool uart_mux_get_state(uart_mux_t *mux) {
    return mux->state;
}