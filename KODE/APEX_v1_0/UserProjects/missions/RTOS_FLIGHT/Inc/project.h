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


typedef struct __attribute__((__packed__)) data_t {
	float3_ts_t acc;
	bool 		launch_detected;
} data_t;


typedef struct TASK_Main_ARGS {
} TASK_Main_ARGS;

TASK_POOL_CONFIGURE(TASK_Main, 1, 8192)
void TASK_Main(void *argument);


typedef struct TASK_DetectLaunch_ARGS {
	data_topic_t	**data_topic;		/**< Pointer to data topic for publishing launch detection status */
	bool			 *launch_signal;	/**< Pointer to boolean flag indicating whether launch has been detected, to be updated by this task */
	osEventFlagsId_t  done_flags;		/**< Event flags ID indicating completion */
} TASK_DetectLaunch_ARGS;
TASK_POOL_CONFIGURE(TASK_DetectLaunch, 1, 576)
void TASK_DetectLaunch(void *argument);


typedef struct TASK_SaveDataDetectLaunch_ARGS {
	data_topic_t		**data_topic;		/**< Acc data topic */
	flash_chunk_ptr_t	 *flash_ptr;		/**< Flash chunk ptr reporting for managing flash writes (and reads) */
	bool				 *launch_signal;	/**< Pointer to boolean flag indicating whether launch has been detected */	
	osEventFlagsId_t      done_flags;		/**< Event flags ID indicating completion */ 
} TASK_SaveDataDetectLaunch_ARGS;
TASK_POOL_CONFIGURE(TASK_SaveDataDetectLaunch, 1, 4096)
void TASK_SaveDataDetectLaunch(void *argument);


typedef struct TASK_SaveData_ARGS {
	data_topic_t		**data_topic;		/**< Pointer to the data topic */
	flash_chunk_ptr_t	 *flash_ptr;		/**< Pointer to flash chunk pointer for managing flash writes (and reads) */
	bool				 *launch_signal;	/**< Pointer to boolean flag indicating whether launch has been detected */	
	osEventFlagsId_t      done_flags;		/**< Event flags ID indicating completion */ 
} TASK_SaveData_ARGS;
TASK_POOL_CONFIGURE(TASK_SaveData, 1, 4096)
void TASK_SaveData(void *argument);


typedef struct TASK_Cleanup_ARGS {
	osSemaphoreId_t		  block_sem_id;	/**< Semaphore indicating 256b has been written */
} TASK_Cleanup_ARGS;


typedef struct TASK_led_rgb_wave_ARGS {
    led_rgb_t			 *led;
    waveform_space_t	**waveform;
	StaticSemaphore_t	 *sem_cd;
	osSemaphoreId_t		 *sem;
	uint32_t			  update_delay_ms;
} TASK_led_rgb_wave_ARGS;
TASK_POOL_CONFIGURE(TASK_led_rgb_wave, 1, 2048);
void TASK_led_rgb_wave(void *args);





#endif // PROJECT_H