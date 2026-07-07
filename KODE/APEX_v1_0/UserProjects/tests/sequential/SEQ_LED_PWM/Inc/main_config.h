// main_config.h
// This file contains the main configuration for the APEX project. It allows you to select one
// of the predefined configuration profiles and enable/disable specific modules.

#ifndef MAIN_CONFIG_H
#define MAIN_CONFIG_H

// =======================================================================
// Schedule profiles (only one can be set to 1)
// =======================================================================
#define APEX_CFG_SCHED_SEQ				1 // Sequential scheduler (no RTOS)
#define APEX_CFG_SCHED_RTOS				0 // RTOS-based scheduler

// =======================================================================
// Profile selection (only one can be set to 1)
// =======================================================================
#define APEX_CFG_PROFILE_TEST			1 // Test profile (for unit testing and development)
#define APEX_CFG_PROFILE_MISSION		0 // Mission profile (for actual deployment)

//========================================================================
// Modules enable/disable (set to 1 to enable, 0 to disable)
//========================================================================
#define APEX_ENABLE_ADXL345				0
#define APEX_ENABLE_BMI088				0
#define APEX_ENABLE_BMP388				0
#define APEX_ENABLE_BUZZER				0
#define APEX_ENABLE_GPS					0
#define APEX_ENABLE_LED					1
#define APEX_ENABLE_LSM303AGR			0
#define APEX_ENABLE_SX127X_1			0
#define APEX_ENABLE_SX127X_2			0
#define APEX_ENABLE_W25Q512				0
#define APEX_ENABLE_WT901B				0


#endif // MAIN_CONFIG_H