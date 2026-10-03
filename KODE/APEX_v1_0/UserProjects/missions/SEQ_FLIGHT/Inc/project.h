#ifndef PROJECT_H
#define PROJECT_H

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "float3.h"
#include "main_config.h"
#include "drivers_config.h"

#include "data_topic.h"
#include "flash_chunk.h"
#include "waveform.h"
#include "tools.h"

#include "event_uart.h"

#include <stdint.h>


// Setup fonction is used any additional execution that needs to be done once at
// the start of the program, after all initializations. It is called once in the main function
// after all peripheral and driver initializations, and before the main loop starts.
// It is typically used to create tasks, initialize variables, or perform any setup that requires
// the drivers to be initialized first.
void setup(void);

#if (APEX_CFG_SCHED_SEQ == 1)

// Loop function is used for the main execution of the program. It is called repeatedly in the main
// function after setup() is called. It contains the main logic of the program, and can be used to
// run tasks, read sensors, transmit data, etc.
// NOTE: If using an RTOS, the main loop might be empty and the logic will be implemented in tasks instead.
// In that case, this function can be left empty or used for any non-RTOS related logic that needs to
// run continuously.
void loop(void);

#endif /* APEX_CFG_SCHED_SEQ == 1 */



typedef struct Apex_data_preamble_t {
	uint8_t id_device;
	uint8_t id_msg;
	uint32_t timestamp;
} Apex_data_preamble_t;



typedef struct Apex_data_lora_gps_t {
	float longitude;
	float latitude;
	float altitude;
} Apex_data_lora_gps_t;

typedef struct Apex_data_lora_event_t {
	event_uart_payload_u payload; 
} Apex_data_lora_event_t;

typedef enum Apex_data_lora_type_t {
	APEX_DATA_LORA_TYPR_GPS		= 0,
	APEX_DATA_LORA_TYPE_EVENT	= 1
} Apex_data_lora_type_t;

typedef union Apex_data_lora_t {
	Apex_data_lora_gps_t gps;
	Apex_data_lora_event_t event;
} Apex_data_lora_t;



typedef struct Apex_data_sensors_t {
	uint32_t time;
	float3_t acc;
	float3_t gyr;
	float baro_press;
	float baro_temp;
} Apex_data_sensors_t;



typedef enum Apex_data_payload_type_t {
	APEX_DATA_PAYLOAD_TYPE_LORA		= 0,
	APEX_DATA_PAYLOAD_TYPE_SENSORS	= 1,
} Apex_data_payload_type_t;

typedef union Apex_data_payload_t {
	Apex_data_lora_t lora;
	Apex_data_sensors_t sensors;
} Apex_data_payload_t;

typedef struct Apex_data_t {
	Apex_data_preamble_t preamble;
	Apex_data_payload_t payload;
} Apex_data_t;



typedef enum Apex_Flight_Phase_t {
	APEX_FLIGHT_PHASE_INIT = 0,
	APEX_FLIGHT_PHASE_PRE_LAUNCH,
	APEX_FLIGHT_PHASE_LAUNCH_DETECTED,
	APEX_FLIGHT_PHASE_POST_LAUNCH
} Apex_Flight_Phase_t;



#endif // PROJECT_H