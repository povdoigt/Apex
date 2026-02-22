// driver_config.h
// This file contains the configuration for the hardware drivers used in the APEX project.

#ifndef DRIVERS_CONFIG_H
#define DRIVERS_CONFIG_H

#include "config/main_config.h"


// =======================================================================
// ADXL345 configuration
// =======================================================================
#if (APEX_ENABLE_ADXL345 == 1)

#include "drivers/ADXL375.h"
extern adxl375_t ADXL375;

#endif

// =======================================================================
// BMI088 configuration
// =======================================================================
#if (APEX_ENABLE_BMI088 == 1)

#include "drivers/BMI088.h"
extern bmi088_t bmi088;

#endif


// =======================================================================
// BMP388 configuration
// =======================================================================
#if (APEX_ENABLE_BMP388 == 1)

#include "drivers/BMP388.h"
extern BMP388_HandleTypeDef bmp388;

#endif


// =======================================================================
// Buzzer configuration
// =======================================================================
#if (APEX_ENABLE_BUZZER == 1)

#include "drivers/buzzer.h"
extern buzzer_t buzzer;

#endif


// =======================================================================
// GPS configuration
// =======================================================================
#if (APEX_ENABLE_GPS == 1)

#include "drivers/gps.h"

#endif


// =======================================================================
// LED configuration
// =======================================================================
#if (APEX_ENABLE_LED == 1)

#include "drivers/led.h"

#endif


// =======================================================================
// LSM303AGR configuration
// =======================================================================
#if (APEX_ENABLE_LSM303AGR == 1)

#include "drivers/LSM303AGR.h"
extern lsm303agr_t lsm303agr;

#endif


// =======================================================================
// SX127x 1 configuration
// =======================================================================
#if (APEX_ENABLE_SX127X_1 == 1)

#include "drivers/sx127x.h"
extern sx127x_t sx127x_1;

#endif

// =======================================================================
// SX127x 2 configuration
// =======================================================================
#if (APEX_ENABLE_SX127X_2 == 1)

#include "drivers/sx127x.h"
extern sx127x_t sx127x_2;

#endif


// =======================================================================
// W25Q512 configuration
// =======================================================================
#if (APEX_ENABLE_W25Q512 == 1)

#include "drivers/w25q_mem.h"
extern W25Q_t w25q;

#endif


// =======================================================================
// WT901B configuration
// =======================================================================
#if (APEX_ENABLE_WT901B == 1)

#include "drivers/WT901B.h"

#endif





// =======================================================================
// Initialization functions prototypes
// =======================================================================
void Drivers_Init(void);


#endif // DRIVERS_CONFIG_H
