#include "project.h"

#include "cmsis_gcc.h"
#include "i2c.h"
#include "i2c_mux.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_i2c.h"

#include <stdint.h>
#include <stdio.h>


void setup(void) {
    uint8_t addr = 0b1011100;
    uint8_t data;

    i2c_mux_set_channel(&hi2c3, I2C_MUX_CHANNEL_0);
    HAL_Delay(1);
    data = 0;
    HAL_I2C_Mem_Read(&hi2c3, addr << 1, 0x0f, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
    __NOP();

    i2c_mux_set_channel(&hi2c3, I2C_MUX_CHANNEL_1);
    HAL_Delay(1);
    data = 0;
    HAL_I2C_Mem_Read(&hi2c3, addr << 1, 0x0f, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
    __NOP();
}

void loop(void) {

}
