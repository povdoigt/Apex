// driver_config.h
// This file contains the configuration for the hardware drivers used in the APEX project.

#include "config/main_config.h"


// =======================================================================
// ADXL345 configuration
// =======================================================================
#if (APEX_ENABLE_ADXL345 == 1)

#include "drivers/ADXL375.h"
#define ADXL375_SPI_HANDLE		hspi1
#define ADXL375_CS_PIN			GPIO_PIN_3
#define ADXL375_CS_PORT			GPIOA
extern adxl375_t ADXL375;

// ... add any additional configuration or definitions for the ADXL345 here

#endif

// =======================================================================
// BMI088 configuration
// =======================================================================
#if (APEX_ENABLE_BMI088 == 1)

#include "drivers/BMI088.h"
extern const bmi_config_t BMI088_CONFIG;
#define BMI088_SPI_HANDLE		hspi1
#define BMI088_ACC_CS_PIN		GPIO_PIN_4
#define BMI088_ACC_CS_PORT		GPIOA
#define BMI088_GYR_CS_PIN		GPIO_PIN_2
#define BMI088_GYR_CS_PORT		GPIOB
extern bmi088_t BMI088;

// ... add any additional configuration or definitions for the BMI088 here

#endif


// =======================================================================
// BMP388 configuration
// =======================================================================
#if (APEX_ENABLE_BMP388 == 1)

#include "drivers/BMP388.h"
extern BMP388_HandleTypeDef bmp388;

// ... add any additional configuration or definitions for the BMP388 here

#endif














#if (APEX_ENABLE_BUZZER == 1)
#include "drivers/buzzer.h"
#endif
#if (APEX_ENABLE_GPS == 1)
#include "drivers/gps.h"
#endif
#if (APEX_ENABLE_LED == 1)
#include "drivers/led.h"
#endif
#if (APEX_ENABLE_LSM303AGR == 1)
#include "drivers/LSM303AGR.h"
#endif
#if (APEX_ENABLE_SX127X_1 == 1 || APEX_ENABLE_SX127X_2 == 1)
#include "drivers/rfm96w.h"
#endif
#if (APEX_ENABLE_W25Q512 == 1)
#include "drivers/w25q_mem.h"
#endif
#if (APEX_ENABLE_WT901B == 1)
#include "drivers/WT901B.h"
#endif