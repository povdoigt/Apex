// drivers_config.h  (copied from project template – voir SEQ_CB_USB)

#ifndef DRIVERS_CONFIG_H
#define DRIVERS_CONFIG_H

#include "main_config.h"

// =======================================================================
// ADXL345
// =======================================================================
#if (APEX_ENABLE_ADXL345 == 1)
#include "ADXL375.h"
extern adxl375_t ADXL375;
#endif

// =======================================================================
// BMI088
// =======================================================================
#if (APEX_ENABLE_BMI088 == 1)
#include "BMI088.h"
extern bmi088_t bmi088;
extern const bmi_config_t bmi088_config;
#if (APEX_CFG_PROFILE_TEST == 1)
#include "BMI088_seq_test.h"
#endif
#endif

// =======================================================================
// BMP388
// =======================================================================
#if (APEX_ENABLE_BMP388 == 1)
#include "BMP388.h"
extern BMP388_HandleTypeDef bmp388;
#endif

// =======================================================================
// Buzzer
// =======================================================================
#if (APEX_ENABLE_BUZZER == 1)
#include "buzzer.h"
extern buzzer_t buzzer;
#endif

// =======================================================================
// GPS
// =======================================================================
#if (APEX_ENABLE_GPS == 1)
#include "gps.h"
#endif

// =======================================================================
// LED
// =======================================================================
#if (APEX_ENABLE_LED == 1)
#include "led.h"
#endif

// =======================================================================
// LSM303AGR
// =======================================================================
#if (APEX_ENABLE_LSM303AGR == 1)
#include "LSM303AGR.h"
extern lsm303agr_t lsm303agr;
#endif

// =======================================================================
// SX127x 1
// =======================================================================
#if (APEX_ENABLE_SX127X_1 == 1)
#include "sx127x.h"
extern sx127x_t sx127x_1;
#endif

// =======================================================================
// SX127x 2
// =======================================================================
#if (APEX_ENABLE_SX127X_2 == 1)
#include "sx127x.h"
extern sx127x_t sx127x_2;
#endif

// =======================================================================
// W25Q512
// =======================================================================
#if (APEX_ENABLE_W25Q512 == 1)
#include "w25q.h"
extern W25Q_t w25q;
#if (APEX_CFG_PROFILE_TEST == 1)
#include "w25q_seq_test.h"
#endif
#endif

// =======================================================================
// WT901B
// =======================================================================
#if (APEX_ENABLE_WT901B == 1)
#include "WT901B.h"
#endif

// =======================================================================
// Prototype d'initialisation séquentielle
// =======================================================================
typedef struct DRIVERS_CONFIG_init_result_t {
#if (APEX_ENABLE_BMI088 == 1)
    BMI_STATE bmi088_init_res;
#endif
#if (APEX_ENABLE_SX127X_1 == 1)
    sx127x_status_t sx127x_1_init_res;
#endif
#if (APEX_ENABLE_SX127X_2 == 1)
    sx127x_status_t sx127x_2_init_res;
#endif
#if (APEX_ENABLE_W25Q512 == 1)
    W25Q_STATE w25q_init_res;
#endif
} DRIVERS_CONFIG_init_result_t;
void DRIVERS_CONFIG_init_seq(DRIVERS_CONFIG_init_result_t *result);

#endif // DRIVERS_CONFIG_H
