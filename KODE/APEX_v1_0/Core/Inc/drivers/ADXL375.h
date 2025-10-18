#ifndef ADXL375_H
#define ADXL375_H

#include "stm32f4xx_hal.h"
#include "peripherals/spi.h"
#include "utils/scheduler.h"


#define ADXL375_CHIP_ID                 0b11100101 // Chip ID


#define ADXL375_REG_DEVID               0x00 // Device ID
#define ADXL375_REG_THRESH_SHOCK        0x1D // Shock threshold
#define ADXL375_REG_OFSX                0x1E // X-axis offset
#define ADXL375_REG_OFSY                0x1F // Y-axis offset
#define ADXL375_REG_OFSZ                0x20 // Z-axis offset
#define ADXL375_REG_DUR                 0x21 // Shock duration
#define ADXL375_REG_LATENT              0x22 // Shock latency
#define ADXL375_REG_WINDOW              0x23 // Shock window
#define ADXL375_REG_THRESH_ACT          0x24 // Activity threshold
#define ADXL375_REG_THRESH_INACT        0x25 // Inactivity threshold
#define ADXL375_REG_TIME_INACT          0x26 // Inactivity time
#define ADXL375_REG_ACT_INACT_CTL       0x27 // Axis enable control for activity/inactivity detection
#define ADXL375_REG_SHOCK_STATUS        0x2A // Axis control for single shock/double shock
#define ADXL375_REG_ACT_SHOCK_STATUS    0x2B // Source of single shock/double shock
#define ADXL375_REG_BW_RATE             0x2C // Data rate and power mode control
#define ADXL375_REG_POWER_CTL           0x2D // Power-saving features control
#define ADXL375_REG_INT_ENABLE          0x2E // Interrupt enable control
#define ADXL375_REG_INT_MAP             0x2F // Interrupt mapping control
#define ADXL375_REG_INT_SOURCE          0x30 // Interrupt source
#define ADXL375_REG_DATA_FORMAT         0x31 // Data format control
#define ADXL375_REG_DATAX0              0x32 // X-axis data 0
#define ADXL375_REG_DATAX1              0x33 // X-axis data 1
#define ADXL375_REG_DATAY0              0x34 // Y-axis data 0
#define ADXL375_REG_DATAY1              0x35 // Y-axis data 1
#define ADXL375_REG_DATAZ0              0x36 // Z-axis data 0
#define ADXL375_REG_DATAZ1              0x37 // Z-axis data 1
#define ADXL375_REG_FIFO_CTL            0x38 // FIFO control
#define ADXL375_REG_FIFO_STATUS         0x39 // FIFO status

#define ADXL375_READ_REG                0b10000000 // Read register command
#define ADXL375_WRITE_REG               0b00000000 // Write register command
#define ADXL375_MULTIPLE_BYTE           0b01000000 // Multiple byte read command
#define ADXL375_SINGLE_BYTE             0b00000000 // Single byte read command

typedef struct ADXL375 {
    SPI_HandleTypeDef *spiHandle;
    GPIO_TypeDef *csPinBank;
    uint16_t csPin;

    float accel_mps2[3];
    float accel_mps2_offset[3];

    bool new_data;
} ADXL375;





void ADXL375_Init(ADXL375 *adxl375,
                  SPI_HandleTypeDef *spiHandle,
                  GPIO_TypeDef *csPinBank,
                  uint16_t csPin);

void ADXL375_ReadRegister(ADXL375 *adxl375, uint8_t reg, uint8_t *value);
void ADXL375_WriteRegister(ADXL375 *adxl375, uint8_t reg, uint8_t value);













































#endif // ADXL375_H
