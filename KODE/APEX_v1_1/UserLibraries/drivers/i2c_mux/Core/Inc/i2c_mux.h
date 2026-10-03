#ifndef I2C_MUX_H
#define I2C_MUX_H

#include "stm32f4xx_hal.h"


#define I2C_MUX_I2C_ADDRESS 0b1110000

#define I2C_MUX_CHANNEL_0_BIT   0b00000000
#define I2C_MUX_CHANNEL_1_BIT   0b00000001
// 0b00000010 reserved
#define I2C_MUX_ENABLE_BIT      0b00000100
// all other bits are reserved

typedef enum i2c_mux_channel_t {
    I2C_MUX_CHANNEL_0 = 0,
    I2C_MUX_CHANNEL_1 = 1
} i2c_mux_channel_t;

void i2c_mux_set_channel(I2C_HandleTypeDef *hi2c, i2c_mux_channel_t channel);

#endif /* I2C_MUX_H */