#include "drivers/ADXL375.h"



void ADXL375_Init(ADXL375 *adxl375,
                  SPI_HandleTypeDef *spiHandle,
                  GPIO_TypeDef *csPinBank,
                  uint16_t csPin) {
    adxl375->spiHandle = spiHandle;
    adxl375->csPinBank = csPinBank;
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
    }

}


void ADXL375_ReadRegister(ADXL375 *adxl375, uint8_t reg, uint8_t *value) {
    uint8_t txBuf[2] = {ADXL375_READ_REG | reg, 0x00};
    uint8_t rxBuf[2];

    HAL_GPIO_WritePin(adxl375->csPinBank, adxl375->csPin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_SPI_TransmitReceive(adxl375->spiHandle, txBuf, rxBuf, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(adxl375->csPinBank, adxl375->csPin, GPIO_PIN_SET);

    *value = rxBuf[1];
}

void ADXL375_WriteRegister(ADXL375 *adxl375, uint8_t reg, uint8_t value) {
    uint8_t txBuf[2] = {ADXL375_WRITE_REG | reg, value};

    HAL_GPIO_WritePin(adxl375->csPinBank, adxl375->csPin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(adxl375->spiHandle, txBuf, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(adxl375->csPinBank, adxl375->csPin, GPIO_PIN_SET);
}
