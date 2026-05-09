// main_config.h
#ifndef MAIN_CONFIG_H
#define MAIN_CONFIG_H

// =======================================================================
// Schedule profiles (only one can be set to 1)
// =======================================================================
#define APEX_CFG_SCHED_SEQ              1
#define APEX_CFG_SCHED_RTOS             0

// =======================================================================
// Profile selection (only one can be set to 1)
// =======================================================================
#define APEX_CFG_PROFILE_TEST           1
#define APEX_CFG_PROFILE_MISSION        0

//========================================================================
// Modules enable / disable
//========================================================================
#define APEX_ENABLE_ADXL345             0
#define APEX_ENABLE_BMI088              0
#define APEX_ENABLE_BMP388              0
#define APEX_ENABLE_BUZZER              0
#define APEX_ENABLE_GPS                 0
#define APEX_ENABLE_LED                 0
#define APEX_ENABLE_LSM303AGR           0
#define APEX_ENABLE_SX127X_1            0
#define APEX_ENABLE_SX127X_2            1
#define APEX_ENABLE_W25Q512             0
#define APEX_ENABLE_WT901B              1

#endif // MAIN_CONFIG_H
