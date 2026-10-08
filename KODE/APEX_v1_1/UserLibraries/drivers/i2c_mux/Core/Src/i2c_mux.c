#include "i2c_mux.h"

void i2c_mux_set_channel(I2C_HandleTypeDef *hi2c, i2c_mux_channel_t channel) {
    uint8_t command = (channel == I2C_MUX_CHANNEL_1 ? I2C_MUX_CHANNEL_1_BIT : I2C_MUX_CHANNEL_0_BIT) | I2C_MUX_ENABLE_BIT;
    HAL_I2C_Master_Transmit(hi2c, I2C_MUX_I2C_ADDRESS << 1, &command, 1, HAL_MAX_DELAY);
}   
