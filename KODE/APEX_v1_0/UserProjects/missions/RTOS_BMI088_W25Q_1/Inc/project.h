#ifndef PROJECT_H
#define PROJECT_H

#include "FreeRTOS.h"
#include "main_config.h"
#include "drivers_config.h"

#include "data_topic.h"
#include "waveform.h"
#include "tools.h"

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


typedef enum flash_data_type_t {
	FLASH_DATA_TYPE_ACC		= 0,
	FLASH_DATA_TYPE_GYR		= 1,
	FLASH_DATA_TYPE_TEMP	= 2,
} flash_data_type_t;


typedef struct flash_data_t {
	flash_data_type_t type;
	union {
		float3_ts_t acc;
		float3_ts_t gyr;
		float_ts_t temp;
	} data;
} flash_data_t;


typedef struct TASK_Main_ARGS {
	void *dummy; // dummy arg to avoid empty struct
} TASK_Main_ARGS;

TASK_POOL_CONFIGURE(TASK_Main, 1, 4096)
void TASK_Main(void *argument);


typedef struct TASK_SaveData_ARGS {
	data_topic_t        **data_topic;	/**< Pointer to the data topic */
	flash_data_type_t     data_type;	/**< Type of data to save (e.g., accelerometer, gyroscope, temperature) */
	osEventFlagsId_t      done_flags;	/**< Optional event flag set on completion (bit 0) */
} TASK_SaveData_ARGS;
TASK_POOL_CONFIGURE(TASK_SaveData, 3, 2048)
void TASK_SaveData(void *argument);


typedef struct TASK_led_rgb_wave_ARGS {
    led_rgb_t			 *led;
    waveform_space_t	**waveform;
	StaticSemaphore_t	 *sem_cd;
	osSemaphoreId_t		 *sem;
	uint32_t			  update_delay_ms;
} TASK_led_rgb_wave_ARGS;
TASK_POOL_CONFIGURE(TASK_led_rgb_wave, 1, 512);
void TASK_led_rgb_wave(void *args);





#endif // PROJECT_H