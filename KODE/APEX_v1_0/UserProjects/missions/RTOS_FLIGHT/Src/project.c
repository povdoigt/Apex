#include "project.h"

#include "FreeRTOS.h"
#include "circular_buffer.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"

#include "data_topic.h"
#include "flash_chunk.h"
#include "float3.h"
#include "scheduler.h"
#include "w25q.h"
#include "waveform.h"

#include "tools.h"
#include "usbd_cdc_if.h"
#include "waveform.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>


/* Mission-wide shared state */

waveform_space_t wait_waveform, launch_waveform, done_waveform;

StaticSemaphore_t waveform_sem_cb;
osSemaphoreId_t waveform_sem_id = NULL;
waveform_space_t *led_waveform = NULL;

data_topic_t *data_topic_acc_ptr = NULL;

flash_chunk_t detect_launch_chunk;
flash_chunk_ptr_t detect_launch_chunk_writer;

flash_chunk_t data_chunk;
flash_chunk_ptr_t data_chunk_writer;

bool launch_detected = false;

#define EVENT_FLAGS_CLEAR_MASK      (0xFFFFFFFFU)
#define EVENT_FLAGS_DONE_BIT        (0x01U)
#define SENSOR_READ_DELAY_MS        (10U)
#define TOPIC_READY_POLL_DELAY_MS   (10U)
#define CDC_READY_POLL_DELAY_MS     (10U)
#define CDC_TX_PACE_DELAY_MS        (1U)
#define SAVE_DURATION_MS            (10000U)
#define USB_BUFFER_LEN              (128U)

#define NBR_DATA_BEFORE_LAUNCH		(50U)	// Number of data points to save before launch detection (for pre-launch context)
#define NBR_DATA_AFTER_LAUNCH		(50U)	// Number of data points to save after launch detection (for post-launch context)

#define FLASH_CHUNK_LAUNCH_DETECTION_SIZE	(0x1000) // 4 KB
#define FLASH_CHUNK_DATA_SIZE				(0x10000) // 64 KB

/* Create and clear a static event-flag object. */
static osEventFlagsId_t init_event_flags(const char *name, StaticEventGroup_t *event_group_storage) {
	osEventFlagsId_t event_flags_id = osEventFlagsNew(&(const osEventFlagsAttr_t){
		.name = name,
		.cb_mem = event_group_storage,
		.cb_size = sizeof(*event_group_storage)
	});

	if (event_flags_id != NULL) {
		osEventFlagsClear(event_flags_id, EVENT_FLAGS_CLEAR_MASK);
	}

	return event_flags_id;
}

/* Initialize flash chunks. */
static void init_flash_chunks(void) {
	flash_chunk_init(&detect_launch_chunk, "DetectLaunchChunk", 0, FLASH_CHUNK_LAUNCH_DETECTION_SIZE);
	flash_chunk_ptr_init(&detect_launch_chunk_writer, &detect_launch_chunk, FLASH_CHUNK_PTR_TYPE_BOUNDEDARY);

	flash_chunk_init(&data_chunk, "DataChunk", FLASH_CHUNK_LAUNCH_DETECTION_SIZE, FLASH_CHUNK_DATA_SIZE);
	flash_chunk_ptr_init(&data_chunk_writer, &data_chunk, FLASH_CHUNK_PTR_TYPE_BOUNDEDARY);
}


/* Spawn async BMI088 init task and return its done-flag ID. */
static osEventFlagsId_t spawn_bmi088_init(BMI_STATE *state, StaticEventGroup_t *done_flags_storage) {
	osEventFlagsId_t done_flags_id = init_event_flags("BMI_Init_Done_Flags", done_flags_storage);

	TASK_BMI088_Init_ARGS bmi_init_args = {
		.imu = &bmi088,
		.hspi = &hspi1,
		.cs_acc_pin = GPIO_PIN_4,
		.cs_acc_bank = GPIOA,
		.cs_gyr_pin = GPIO_PIN_2,
		.cs_gyr_bank = GPIOB,
		.cfg = &bmi088_config,
		.return_state = state,
		.done_flags = done_flags_id
	};

	osThreadAttr_t bmi_init_attr = {
		.name = "TASK_BMI088_Init",
		.priority = (osPriority_t)osPriorityNormal,
	};

	OS_THREAD_NEW_CSTM(TASK_BMI088_Init, bmi_init_args, bmi_init_attr, osWaitForever);
	return done_flags_id;
}

/* Spawn async W25Q init task and return its done-flag ID. */
static osEventFlagsId_t spawn_w25q_init(W25Q_STATE *w25q_state, StaticEventGroup_t *done_flags_storage) {
	osEventFlagsId_t done_flags_id = init_event_flags("W25Q_Init_Done_Flags", done_flags_storage);

	TASK_W25Q_Init_ARGS w25q_init_args = {
		.chip = &w25q,
		.hspi = &hspi2,
		.cs_bank = GPIOC,
		.cs_pin = GPIO_PIN_1,
		.result = w25q_state,
		.done_flags = done_flags_id
	};

	osThreadAttr_t w25q_init_attr = {
		.name = "TASK_W25Q_Init",
		.priority = (osPriority_t)osPriorityNormal,
	};

	OS_THREAD_NEW_CSTM(TASK_W25Q_Init, w25q_init_args, w25q_init_attr, osWaitForever);
	return done_flags_id;
}

/* Erase first and second flash block once W25Q init has completed. */
static W25Q_STATE run_flash_erase_blocks(void) {
	W25Q_STATE w25q_state_1 = W25Q_PARAM_ERR;
	W25Q_STATE w25q_state_2 = W25Q_PARAM_ERR;

	StaticEventGroup_t done_flags_1, done_flags_2;
	osEventFlagsId_t done_flags_1_id = init_event_flags("EraseFlashDoneFlags_1", &done_flags_1);
	osEventFlagsId_t done_flags_2_id = init_event_flags("EraseFlashDoneFlags_2", &done_flags_2);

	TASK_W25Q_SendCmdAddr_ARGS erase_args_1 = {
		.chip = &w25q,
		.cmd = W25Q_64KB_BLOCK_ERASE_4B,
		.addr = 0,
		.result = &w25q_state_1,
		.done_flags = done_flags_1_id
	};
	TASK_W25Q_SendCmdAddr_ARGS erase_args_2 = {
		.chip = &w25q,
		.cmd = W25Q_64KB_BLOCK_ERASE_4B,
		.addr = 0x10000, // next block
		.result = &w25q_state_2,
		.done_flags = done_flags_2_id
	};

	osThreadAttr_t erase_attr_1 = {
		.name = "TASK_W25Q_EraseSector_1",
		.priority = (osPriority_t)osPriorityNormal,
	};
	osThreadAttr_t erase_attr_2 = {
		.name = "TASK_W25Q_EraseSector_2",
		.priority = (osPriority_t)osPriorityNormal,
	};

	OS_THREAD_NEW_CSTM(TASK_W25Q_SendCmdAddr, erase_args_1, erase_attr_1, osWaitForever);
	OS_THREAD_NEW_CSTM(TASK_W25Q_SendCmdAddr, erase_args_2, erase_attr_2, osWaitForever);

	osEventFlagsWait(done_flags_1_id, EVENT_FLAGS_DONE_BIT, osFlagsWaitAll, osWaitForever);
	osEventFlagsWait(done_flags_2_id, EVENT_FLAGS_DONE_BIT, osFlagsWaitAll, osWaitForever);

	if (w25q_state_1 != W25Q_OK) {
		return w25q_state_1;
	}
	if (w25q_state_2 != W25Q_OK) {
		return w25q_state_2;
	}

	return W25Q_WaitForReady_RTOS(&w25q);
}

/* Start periodic sensor acquisition tasks. */
static void spawn_measurement_tasks(data_topic_t **acc_topic_ptr) {
	TASK_BMI088_ReadAcc_ARGS bmi_acc_args = {
		.imu = &bmi088,
		.dt = acc_topic_ptr,
		.delay_ms = SENSOR_READ_DELAY_MS,
		.return_state = NULL
	};
	osThreadAttr_t bmi_acc_attr = {
		.name = "TASK_BMI088_ReadAcc",
		.priority = (osPriority_t)osPriorityNormal,
	};
	OS_THREAD_NEW_CSTM(TASK_BMI088_ReadAcc, bmi_acc_args, bmi_acc_attr, osWaitForever);
}

/* Spawn the launch detection task. */
static void spawn_detect_launch_task(data_topic_t **acc_topic_ptr, bool *launch_signal_ptr, osEventFlagsId_t done_flags_id) {
	TASK_DetectLaunch_ARGS detect_launch_args = {
		.data_topic = acc_topic_ptr,
		.launch_signal = launch_signal_ptr,
		.done_flags = done_flags_id
	};
	osThreadAttr_t detect_launch_attr = {
		.name = "TASK_DetectLaunch",
		.priority = (osPriority_t)osPriorityNormal,
	};
	OS_THREAD_NEW_CSTM(TASK_DetectLaunch, detect_launch_args, detect_launch_attr, osWaitForever);
}

static void spawn_save_data_detect_launch_task(data_topic_t **acc_topic_ptr, flash_chunk_ptr_t *flash_ptr, bool *launch_signal_ptr, osEventFlagsId_t done_flags_id) {
	TASK_SaveDataDetectLaunch_ARGS save_data_args = {
		.data_topic = acc_topic_ptr,
		.flash_ptr = flash_ptr,
		.launch_signal = launch_signal_ptr,
		.done_flags = done_flags_id
	};
	osThreadAttr_t save_data_attr = {
		.name = "TASK_SaveDataDetectLaunch",
		.priority = (osPriority_t)osPriorityNormal,
	};
	OS_THREAD_NEW_CSTM(TASK_SaveDataDetectLaunch, save_data_args, save_data_attr, osWaitForever);
}	

/* Start one save task per data stream (ACC, GYR, TEMP, ...). */
static void spawn_save_data_tasks(osEventFlagsId_t save_data_done_id, data_topic_t **data_topic_ptr, flash_chunk_ptr_t *flash_ptr, bool *launch_signal_ptr) {
	TASK_SaveData_ARGS save_data_args = {
		.data_topic = data_topic_ptr,
		.flash_ptr = flash_ptr,
		.launch_signal = launch_signal_ptr,
		.done_flags = save_data_done_id
	};
	osThreadAttr_t save_data_attr = {
		.name = "TASK_SaveAccData",
		.priority = (osPriority_t)osPriorityNormal,
	};
	OS_THREAD_NEW_CSTM(TASK_SaveData, save_data_args, save_data_attr, osWaitForever);
}

/* Read back flash and print decoded frames to USB CDC. */
static void print_flash_contents(flash_chunk_ptr_t *flash_ptr_writer) {
	W25Q_STATE w25q_state = W25Q_PARAM_ERR;

	// Create a reader pointer from the writer pointer passed as argument, to read from the start of the chunk.
	flash_chunk_ptr_t flash_ptr_reader;
	flash_chunk_ptr_init(&flash_ptr_reader, flash_ptr_writer->chunk, FLASH_CHUNK_PTR_TYPE_BOUNDEDARY);

	StaticEventGroup_t read_done_flags;
	osEventFlagsId_t read_done_flags_id = init_event_flags("ReadFlashDoneFlags", &read_done_flags);

	data_t read_data;

	TASK_W25Q_ReadData_ARGS read_args = {
		.chip = &w25q,
		.addr = 0,
		.buffer = (uint8_t *)&read_data,
		.buf_size = sizeof(read_data),
		.result = &w25q_state,
		.done_flags = read_done_flags_id
	};
	osThreadAttr_t read_attr = {
		.name = "TASK_W25Q_ReadData",
		.priority = (osPriority_t)osPriorityNormal,
	};

	char usb_buffer[USB_BUFFER_LEN];

	snprintf(usb_buffer, sizeof(usb_buffer), "Reading flash chunk %s contents...\r\n", flash_ptr_writer->chunk->name);
	CDC_Transmit_FS((uint8_t *)usb_buffer, strlen(usb_buffer));

	while (flash_ptr_reader.offset < flash_ptr_writer->offset) {
		osEventFlagsClear(read_done_flags_id, EVENT_FLAGS_CLEAR_MASK);
		read_args.addr = flash_chunk_ptr_get_and_update(&flash_ptr_reader, sizeof(read_data), NULL);
		OS_THREAD_NEW_CSTM(TASK_W25Q_ReadData, read_args, read_attr, osWaitForever);
		osEventFlagsWait(read_done_flags_id, EVENT_FLAGS_DONE_BIT, osFlagsWaitAll, osWaitForever);

		snprintf(usb_buffer, sizeof(usb_buffer), "LAUNCH %1u    TIME %10lu    ACC X=%+10.3f  Y=%+10.3f  Z=%+10.3f  m/s2\r\n",
				read_data.launch_detected, read_data.acc.ts, read_data.acc.data.x, read_data.acc.data.y, read_data.acc.data.z);

		CDC_Transmit_FS((uint8_t *)usb_buffer, strlen(usb_buffer));
		osDelay(CDC_TX_PACE_DELAY_MS);
	}

	snprintf(usb_buffer, sizeof(usb_buffer), "Finished reading flash chunk contents (%lu bytes).\r\n", flash_ptr_reader.offset);
	CDC_Transmit_FS((uint8_t *)usb_buffer, strlen(usb_buffer));
}

static void change_waveform(waveform_space_t **old_waveform, waveform_space_t *new_waveform, osSemaphoreId_t *sem) {
	osSemaphoreAcquire(*sem, osWaitForever);
	*old_waveform = new_waveform;
	osSemaphoreRelease(*sem);
}


void setup(void) {
	TASK_POOL_CREATE(TASK_BMI088_Init);
	TASK_POOL_CREATE(TASK_BMI088_ReadAcc);
	TASK_POOL_CREATE(TASK_BMI088_ReadGyr);
	TASK_POOL_CREATE(TASK_BMI088_ReadTemp);

	TASK_POOL_CREATE(TASK_W25Q_Init);
	TASK_POOL_CREATE(TASK_W25Q_SendCmdAddr);
	TASK_POOL_CREATE(TASK_W25Q_WriteData);
	TASK_POOL_CREATE(TASK_W25Q_ReadData);

	TASK_POOL_CREATE(TASK_Main);
	TASK_POOL_CREATE(TASK_DetectLaunch);
	TASK_POOL_CREATE(TASK_SaveDataDetectLaunch);
	TASK_POOL_CREATE(TASK_SaveData);
	TASK_POOL_CREATE(TASK_led_rgb_wave);

	osDelay(1000);

	init_flash_chunks();

	osThreadAttr_t main_loop_attr = {
		.name = "TASK_Main",
		.priority = (osPriority_t)osPriorityNormal,
	};
	OS_THREAD_NEW_CSTM(TASK_Main, (TASK_Main_ARGS){}, main_loop_attr, osWaitForever);
}


TASK_POOL_ALLOCATE(TASK_Main);
void TASK_Main(void *argument) {
	(void)argument;

	// ===================================================
	//                  LED initialization
	// ===================================================

	// Waveform to indicate that measurement is in progress.
	// Blue sinusoid with frequency 1 Hz, multiplied by a strobing function to create on-off effect.
	Waveform_Init_Space(&wait_waveform, waveform_space_mult_const, &(waveform_space_mult_const_t){
		.wave_function_mult = waveform_sine,
		.ctx_mult = &(waveform_sine_t){ .start = 0, .end = 1},
		.factor = FLOAT3_UNIT_Z	// Blue channel only
	}, 1000, true);

	// Waveform to indicate that launch has been detected.
	// Two fast strobs dephased on green and blue channels respectively.
	Waveform_Init_Space(&launch_waveform, waveform_space_add, &(waveform_space_add_t){
		.wave_function_add_1 = waveform_space_mult_const,
		.ctx_add_1 = &(waveform_space_mult_const_t){
			.wave_function_mult = waveform_scale_add,
			.ctx_mult = &(waveform_scale_add_t){
				.wave_function_add_1 = waveform_gate,
				.ctx_add_1 = &(waveform_gate_t){ .start = 0.0f, .end = 0.1f },
				.wave_function_add_2 = waveform_gate,
				.ctx_add_2 = &(waveform_gate_t){ .start = 0.2f, .end = 0.3f },
			},
			.factor = FLOAT3_UNIT_Y 
		},
		.wave_function_add_2 = waveform_space_mult_const,
		.ctx_add_2 = &(waveform_space_mult_const_t){
			.wave_function_mult = waveform_scale_add,
			.ctx_mult = &(waveform_scale_add_t){
				.wave_function_add_1 = waveform_gate,
				.ctx_add_1 = &(waveform_gate_t){ .start = 0.05f, .end = 0.15f },
				.wave_function_add_2 = waveform_gate,
				.ctx_add_2 = &(waveform_gate_t){ .start = 0.25f, .end = 0.35f },
			},
			.factor = FLOAT3_UNIT_Z
		}
	}, 1000, false);

	// Waveform change to indicate that saving is done.
	// Fast green strobing.
	Waveform_Init_Space(&done_waveform, waveform_space_mult_const, &(waveform_space_mult_const_t){
		.wave_function_mult = waveform_scale_add,
		.ctx_mult = &(waveform_scale_add_t){
			.wave_function_add_1 = waveform_gate,
			.ctx_add_1 = &(waveform_gate_t){ .start = 0.0f, .end = 0.1f },
			.wave_function_add_2 = waveform_gate,
			.ctx_add_2 = &(waveform_gate_t){ .start = 0.2f, .end = 0.3f },
		},
		.factor = FLOAT3_UNIT_Y	// Green channel only
	}, 1000, true);


	LED_Init(&led0.red  , &DRIVERS_CONFIG_LED0_TIMER_HANDLE, DRIVERS_CONFIG_LED0_R_CHANNEL);
	LED_Init(&led0.green, &DRIVERS_CONFIG_LED0_TIMER_HANDLE, DRIVERS_CONFIG_LED0_G_CHANNEL);
	LED_Init(&led0.blue , &DRIVERS_CONFIG_LED0_TIMER_HANDLE, DRIVERS_CONFIG_LED0_B_CHANNEL);

	TASK_led_rgb_wave_ARGS led_wave_args = {
		.led = &led0,
		.waveform = &led_waveform,
		.sem_cd = &waveform_sem_cb,
		.sem = &waveform_sem_id,
		.update_delay_ms = 10,
	};
	osThreadAttr_t led_wave_attr = {
		.name = "TASK_led_rgb_wave",
		.priority = (osPriority_t)osPriorityNormal,
	};
	OS_THREAD_NEW_CSTM(TASK_led_rgb_wave, led_wave_args, led_wave_attr, osWaitForever);	

	// ===================================================
	//                  Concurrent initialization
	// ===================================================


	BMI_STATE state = BMI_UNKNOWN_ERR;
	StaticEventGroup_t bmi_init_done_flags;
	osEventFlagsId_t bmi_init_done_flags_id = spawn_bmi088_init(&state, &bmi_init_done_flags);

	W25Q_STATE w25q_state = W25Q_PARAM_ERR;
	StaticEventGroup_t w25q_init_done_flags;
	osEventFlagsId_t w25q_init_done_flags_id = spawn_w25q_init(&w25q_state, &w25q_init_done_flags);

	// Keep strict sequencing: erase only after W25Q init is confirmed.
	osEventFlagsWait(w25q_init_done_flags_id, EVENT_FLAGS_DONE_BIT, osFlagsWaitAll, osWaitForever);
	if (w25q_state != W25Q_OK) { osThreadExit_Cstm(); }

	// ===================================================
	//                  Erase first flash block
	// ===================================================

	w25q_state = run_flash_erase_blocks();
	if (w25q_state != W25Q_OK) { osThreadExit_Cstm(); }

	// BMI init runs in parallel and is checked here.
	osEventFlagsWait(bmi_init_done_flags_id, EVENT_FLAGS_DONE_BIT, osFlagsWaitAll, osWaitForever);
	if (state != BMI_OK) { osThreadExit_Cstm(); }

	// Wait for TASK_led_rgb_wave to run at least one cycle before changing its waveform.
	while (waveform_sem_id == NULL) {
		osDelay(TOPIC_READY_POLL_DELAY_MS);
	}

	// ===================================================
	//                  Spawn measurement tasks
	// ===================================================

	spawn_measurement_tasks(&data_topic_acc_ptr);

	// ===================================================
	//  Spawn Detect Launch and save data detection tasks
	// ===================================================

	spawn_detect_launch_task(&data_topic_acc_ptr, &launch_detected, NULL);
	spawn_save_data_detect_launch_task(&data_topic_acc_ptr, &detect_launch_chunk_writer, &launch_detected, NULL);

	// ===================================================
	//          Spawn regular save data task 
	// ===================================================

	StaticEventGroup_t save_data_done_flag;
	osEventFlagsId_t save_data_done_id;
	save_data_done_id = init_event_flags("SaveDataDoneFlag", &save_data_done_flag);
	spawn_save_data_tasks(save_data_done_id, &data_topic_acc_ptr, &data_chunk_writer, &launch_detected);

	// ===================================================
	//		Wait for save to complete and indicate it
	// ===================================================

	change_waveform(&led_waveform, &wait_waveform, &waveform_sem_id);
	
	osEventFlagsWait(save_data_done_id, EVENT_FLAGS_DONE_BIT, osFlagsWaitAll, osWaitForever);

	change_waveform(&led_waveform, &done_waveform, &waveform_sem_id);

	// =========================================================
	//                  Print flash content
	// =========================================================

	// Wait for USB CDC port to be opened before attempting to transmit.
	while (!cdc_port_open) {
		osDelay(CDC_READY_POLL_DELAY_MS);
	}

	// First chunk
	change_waveform(&led_waveform, &wait_waveform, &waveform_sem_id);
	print_flash_contents(&detect_launch_chunk_writer);
	print_flash_contents(&data_chunk_writer);
	change_waveform(&led_waveform, NULL, &waveform_sem_id);

	osThreadExit_Cstm();
}


TASK_POOL_ALLOCATE(TASK_DetectLaunch);
void TASK_DetectLaunch(void *argument) {
	TASK_DetectLaunch_ARGS *args = (TASK_DetectLaunch_ARGS *)argument;
	if (!args->data_topic) {
		osThreadExit_Cstm();
	}

	while (*(args->data_topic) == NULL) {
		// Wait until the producer task publishes the topic pointer.
		osDelay(TOPIC_READY_POLL_DELAY_MS);
	}

	data_sub_t sub = { 0 };
	data_sub_attach(&sub, *(args->data_topic), DATA_ATTACH_FROM_NOW);

	float3_ts_t acc_data = { 0 };

	while (acc_data.data.y < 9.81f * 3.0f) { // Wait for Y-acceleration to exceed ~3g.
		data_sub_wait_for_data(&sub, osWaitForever);
		data_sub_read(&sub, &acc_data);
	}
	*(args->launch_signal) = true;

	change_waveform(&led_waveform, &launch_waveform, &waveform_sem_id);
	osDelay(600);
	change_waveform(&led_waveform, &wait_waveform, &waveform_sem_id);

	if (args->done_flags) { osEventFlagsSet(args->done_flags, EVENT_FLAGS_DONE_BIT); }

	osThreadExit_Cstm();
}


TASK_POOL_ALLOCATE(TASK_SaveDataDetectLaunch);
void TASK_SaveDataDetectLaunch(void *argument) {
	TASK_SaveDataDetectLaunch_ARGS *args = (TASK_SaveDataDetectLaunch_ARGS *)argument;
	if (!args->data_topic || !args->launch_signal) {
		osThreadExit_Cstm();
	}

	data_t data[NBR_DATA_BEFORE_LAUNCH + NBR_DATA_AFTER_LAUNCH] = { 0 };
	data_t *data_before = data;
	data_t *data_after = data + NBR_DATA_BEFORE_LAUNCH;

	cb_status_t status;

	circular_buffer_t data_before_cb;
	status = cb_init(&data_before_cb, data_before, sizeof(data_t), NBR_DATA_BEFORE_LAUNCH, CB_OVERWRITE_OLDEST);
	if (status != CB_OK) {
		osThreadExit_Cstm();
	}

	circular_buffer_t data_after_cb;
	status = cb_init(&data_after_cb, data_after, sizeof(data_t), NBR_DATA_AFTER_LAUNCH, CB_REJECT_NEW);
	if (status != CB_OK) {
		osThreadExit_Cstm();
	}

	while (*(args->data_topic) == NULL) {
		// Wait until the producer task publishes the topic pointer.
		osDelay(TOPIC_READY_POLL_DELAY_MS);
	}

	data_sub_t sub = { 0 };
	data_sub_attach(&sub, *(args->data_topic), DATA_ATTACH_FROM_NOW);

	data_t flash_data;

	size_t nb_data_after = 0;

	bool sample_done = false;


	while (!sample_done) {
		data_sub_wait_for_data(&sub, osWaitForever);
		data_sub_read(&sub, &(flash_data.acc));

		flash_data.launch_detected = *(args->launch_signal);

		if (!*(args->launch_signal)) {
			// Before launch detection, keep filling the pre-launch circular buffer. Once it's full, the oldest
			// data will be overwritten, ensuring we always have the most recent NBR_DATA_BEFORE_LAUNCH samples
			// leading up to the launch.	
			cb_push(&data_before_cb, &flash_data);
		} else {
			// Keep filling the post-launch circular buffer until it's full. Once it's full, stop accepting new
			// data to preserve the context immediately following the launch.
			cb_push(&data_after_cb, &flash_data);
			nb_data_after++;
			if (nb_data_after >= NBR_DATA_AFTER_LAUNCH) {
				sample_done = true;
			}
		}
	}

	// Prepare data buffer to write to flash:
	// - pre-launch samples in chronological order
	// - erase gap between pre and post launch data with memmove
	if (data_before_cb.count < NBR_DATA_BEFORE_LAUNCH) {
		// Not enough pre-launch data collected - this could happen if the launch is detected very early.
		// In this case, we just write whatever we have, without reordering (since we don't have a full buffer).
		// However, a gap will be present between the last pre-launch sample and the first post-launch sample that
		// need to be removed by calling memmove.
		// The post-launch data will still be in chronological order.
		memmove(data_before + data_before_cb.count,
				data_after,
				data_after_cb.count * sizeof(flash_data));	
	} else if (data_before_cb.head != 0){
		// We have a full pre-launch buffer but disordered, so we reorder it to be in chronological order.
		size_t nb_to_move = data_before_cb.capacity - data_before_cb.head;
		size_t head = data_before_cb.head;
		for (size_t i = 0; i < nb_to_move; i++) {
			flash_data = data_before[i];
			data_before[i] = data_before[head + i];
			data_before[head + i] = flash_data;
		}
	}

	// At this point, the 'data' array contains up to NBR_DATA_BEFORE_LAUNCH pre-launch samples followed by up to
	// NBR_DATA_AFTER_LAUNCH post-launch samples, all in chronological order, and with no gaps. We can write this
	// whole buffer to flash in one go.
	// Current thread need to stay alive until the write is done since the write task relies on the data buffer so
	// it needs to stay loaded on the stack.
	W25Q_STATE w25q_state;
	StaticEventGroup_t w25q_done_flags;
	osEventFlagsId_t w25q_done_id = init_event_flags("W25Q_Loop_Write_Done_Flags", &w25q_done_flags);
	TASK_W25Q_WriteData_ARGS w25q_write_args = {
		.chip = &w25q,
		.buffer = (uint8_t *)data,
		.addr = flash_chunk_ptr_get_and_update(args->flash_ptr, sizeof(data), NULL),
		.buf_size = sizeof(data),
		.result = &w25q_state,
		.done_flags = w25q_done_id,
	};
	osThreadAttr_t w25q_write_attr = {
		.name = "TASK_W25Q_WriteDataLoop",
		.priority = (osPriority_t)osPriorityNormal,
	};
	OS_THREAD_NEW_CSTM(TASK_W25Q_WriteData, w25q_write_args, w25q_write_attr, osWaitForever);
	osEventFlagsWait(w25q_done_id, EVENT_FLAGS_DONE_BIT, osFlagsWaitAll, osWaitForever);

	if (args->done_flags) { osEventFlagsSet(args->done_flags, EVENT_FLAGS_DONE_BIT); }
	osThreadExit_Cstm();
}


TASK_POOL_ALLOCATE(TASK_SaveData);
void TASK_SaveData(void *argument) {
	TASK_SaveData_ARGS *args = (TASK_SaveData_ARGS *)argument;
	if (!args->data_topic) {
		osThreadExit_Cstm();
	}

	data_t flash_data;

	W25Q_STATE w25q_state;

	StaticEventGroup_t w25q_done_flags;
	osEventFlagsId_t w25q_done_id = init_event_flags("W25Q_Op_Done_Flags", &w25q_done_flags);

	TASK_W25Q_WriteData_ARGS w25q_write_args = {
		.chip = &w25q,
		.buffer = (uint8_t *)&flash_data,
		.addr = 0,
		.buf_size = sizeof(flash_data),
		.result = &w25q_state,
		.done_flags = w25q_done_id,
	};

	osThreadAttr_t w25q_write_attr = {
		.name = "TASK_W25Q_WriteData",
		.priority = (osPriority_t)osPriorityNormal,
	};

	while (*(args->data_topic) == NULL) {
		// Wait until the producer task publishes the topic pointer.
		osDelay(TOPIC_READY_POLL_DELAY_MS);
	}

	data_sub_t sub = { 0 };
	data_sub_attach(&sub, *(args->data_topic), DATA_ATTACH_FROM_NOW);

	while (!*(args->launch_signal)) {
		// Wait util launch is detected before starting to save regular data flow.
		osDelay(TOPIC_READY_POLL_DELAY_MS);
	}

	uint32_t t0 = osKernelGetTickCount();

	while (osKernelGetTickCount() - t0 < SAVE_DURATION_MS) {
		data_sub_wait_for_data(&sub, osWaitForever);
		data_sub_read(&sub, &(flash_data.acc));

		flash_data.launch_detected = *(args->launch_signal);

		w25q_write_args.addr = flash_chunk_ptr_get_and_update(args->flash_ptr, sizeof(flash_data), NULL);
		osEventFlagsClear(w25q_done_id, EVENT_FLAGS_CLEAR_MASK);
		OS_THREAD_NEW_CSTM(TASK_W25Q_WriteData, w25q_write_args, w25q_write_attr, osWaitForever);
		osEventFlagsWait(w25q_done_id, EVENT_FLAGS_DONE_BIT, osFlagsWaitAll, osWaitForever);
		if (w25q_state != W25Q_OK) {
			flash_chunk_ptr_get_and_update(args->flash_ptr, -sizeof(flash_data), NULL);
			osThreadExit_Cstm();
		}
	}

	if (args->done_flags) { osEventFlagsSet(args->done_flags, EVENT_FLAGS_DONE_BIT); }
	osThreadExit_Cstm();
}


TASK_POOL_ALLOCATE(TASK_led_rgb_wave);
void TASK_led_rgb_wave(void *argument) {
	TASK_led_rgb_wave_ARGS *args = (TASK_led_rgb_wave_ARGS *)argument;
	if (!args->led || !args->waveform) {
		osThreadExit_Cstm();
	}
	if (args->update_delay_ms == 0) {
		args->update_delay_ms = 1; // Default to 10 ms if not set
	}	

	// Create semaphore for synchronizing access to the Waveform
	*(args->sem) = osSemaphoreNew(1, 1, &(osSemaphoreAttr_t){
		.name = "WaveformSem",
		.cb_mem = args->sem_cd,
		.cb_size = sizeof(*(args->sem_cd))
	});

	float3_t color;

	while (1) {
		osSemaphoreAcquire(*(args->sem), osWaitForever);
		if (*(args->waveform) != NULL) {
			color = Waveform_Play_Space(*(args->waveform), osKernelGetTickCount());
		} else {
			color = FLOAT3_ZERO;
		}
		LED_RGB_SetColor(args->led, color);
		osSemaphoreRelease(*(args->sem));
		osDelay(args->update_delay_ms); // Update every 10 ms
	}
}
