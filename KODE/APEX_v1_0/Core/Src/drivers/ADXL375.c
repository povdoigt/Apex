#include "drivers/ADXL375.h"
#include "cmsis_gcc.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_spi.h"



void ADXL375_Init(ADXL375 *adxl375,
                  SPI_HandleTypeDef *spiHandle,
                  GPIO_TypeDef *csPinBank,
                  uint16_t csPin) {
    adxl375->spiHandle = spiHandle;
    adxl375->csPinBank = csPinBank;
    adxl375->csPin = csPin;
    adxl375->csPin = csPin;

    adxl375->accel_mps2[0] = 0.0f;
    adxl375->accel_mps2[1] = 0.0f;
    adxl375->accel_mps2[2] = 0.0f;

    adxl375->accel_mps2_offset[0] = 0.0f;
    adxl375->accel_mps2_offset[1] = 0.0f;
    adxl375->accel_mps2_offset[2] = 0.0f;

    adxl375->new_data = false;


    uint8_t chipID;

    ADXL375_ReadRegister(adxl375, ADXL375_REG_DEVID, &chipID);

    if (chipID != ADXL375_CHIP_ID) {
        // Handle error: chip ID mismatch
        return;
    } else {
        // same !!
        __NOP();
    }

}


void ADXL375_ReadRegister(ADXL375 *adxl375, uint8_t reg, uint8_t *value) {
    uint8_t cmd = ADXL375_READ_REG | reg;

    // Configure SPI settings
    adxl375->spiHandle->Init.CLKPolarity = SPI_POLARITY_HIGH;
    adxl375->spiHandle->Init.CLKPhase = SPI_PHASE_2EDGE;
    HAL_SPI_Init(adxl375->spiHandle);

    HAL_GPIO_WritePin(adxl375->csPinBank, adxl375->csPin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(adxl375->spiHandle, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(adxl375->spiHandle, value, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(adxl375->csPinBank, adxl375->csPin, GPIO_PIN_SET);

    // Restore SPI settings
    adxl375->spiHandle->Init.CLKPolarity = SPI_POLARITY_LOW;
    adxl375->spiHandle->Init.CLKPhase = SPI_PHASE_1EDGE;
    HAL_SPI_Init(adxl375->spiHandle);
}

void ADXL375_WriteRegister(ADXL375 *adxl375, uint8_t reg, uint8_t value) {
    uint8_t txBuf[2] = {ADXL375_WRITE_REG | reg, value};

    HAL_GPIO_WritePin(adxl375->csPinBank, adxl375->csPin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(adxl375->spiHandle, txBuf, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(adxl375->csPinBank, adxl375->csPin, GPIO_PIN_SET);
}
