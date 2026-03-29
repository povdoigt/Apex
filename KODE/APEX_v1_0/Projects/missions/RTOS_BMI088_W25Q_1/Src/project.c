#include "project.h"

#include "FreeRTOS.h"
#include "cmsis_os2.h"

#include "data_topic.h"
#include "float3.h"
#include "scheduler.h"

#include "tools.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* Mission-wide shared state */

uint32_t flash_addr = 0;
StaticSemaphore_t flash_addr_sem_cd;
osSemaphoreId_t flash_addr_sem_id;

data_topic_t *data_topic_acc_ptr = NULL;
data_topic_t *data_topic_gyr_ptr = NULL;
data_topic_t *data_topic_temp_ptr = NULL;

#define EVENT_FLAGS_CLEAR_MASK      (0xFFFFFFFFU)
#define EVENT_FLAGS_DONE_BIT        (0x01U)
#define SENSOR_READ_DELAY_MS        (10U)
#define TOPIC_READY_POLL_DELAY_MS   (10U)
#define CDC_READY_POLL_DELAY_MS     (10U)
#define CDC_TX_PACE_DELAY_MS        (1U)
#define SAVE_TASK_COUNT             (3U)
#define SAVE_DURATION_MS            (10000U)
#define FLASH_ERASE_ADDR            (0U)
#define USB_BUFFER_LEN              (128U)

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

/* Erase first flash block once W25Q init has completed. */
static W25Q_STATE run_flash_erase_first_block(void) {
	W25Q_STATE w25q_state = W25Q_PARAM_ERR;

	StaticEventGroup_t done_flags;
	osEventFlagsId_t done_flags_id = init_event_flags("EraseFlashDoneFlags", &done_flags);

	TASK_W25Q_SendCmdAddr_ARGS erase_args = {
		.chip = &w25q,
		.cmd = W25Q_64KB_BLOCK_ERASE,
		.addr = FLASH_ERASE_ADDR,
		.result = &w25q_state,
		.done_flags = done_flags_id
	};

	osThreadAttr_t erase_attr = {
		.name = "TASK_W25Q_EraseSector",
		.priority = (osPriority_t)osPriorityNormal,
	};

	OS_THREAD_NEW_CSTM(TASK_W25Q_SendCmdAddr, erase_args, erase_attr, osWaitForever);
	osEventFlagsWait(done_flags_id, EVENT_FLAGS_DONE_BIT, osFlagsWaitAll, osWaitForever);
	if (w25q_state != W25Q_OK) {
		return w25q_state;
	}

	return W25Q_WaitForReady_RTOS(&w25q);
}

/* Start periodic sensor acquisition tasks. */
static void spawn_measurement_tasks(void) {
	TASK_BMI088_ReadAcc_ARGS bmi_acc_args = {
		.imu = &bmi088,
		.dt = &data_topic_acc_ptr,
		.delay_ms = SENSOR_READ_DELAY_MS,
		.return_state = NULL
	};
	osThreadAttr_t bmi_acc_attr = {
		.name = "TASK_BMI088_ReadAcc",
		.priority = (osPriority_t)osPriorityNormal,
	};
	OS_THREAD_NEW_CSTM(TASK_BMI088_ReadAcc, bmi_acc_args, bmi_acc_attr, osWaitForever);

	TASK_BMI088_ReadGyr_ARGS bmi_gyr_args = {
		.imu = &bmi088,
		.dt = &data_topic_gyr_ptr,
		.delay_ms = SENSOR_READ_DELAY_MS,
		.return_state = NULL
	};
	osThreadAttr_t bmi_gyr_attr = {
		.name = "TASK_BMI088_ReadGyr",
		.priority = (osPriority_t)osPriorityNormal,
	};
	OS_THREAD_NEW_CSTM(TASK_BMI088_ReadGyr, bmi_gyr_args, bmi_gyr_attr, osWaitForever);

	TASK_BMI088_ReadTemp_ARGS bmi_temp_args = {
		.imu = &bmi088,
		.dt = &data_topic_temp_ptr,
		.delay_ms = SENSOR_READ_DELAY_MS,
		.return_state = NULL
	};
	osThreadAttr_t bmi_temp_attr = {
		.name = "TASK_BMI088_ReadTemp",
		.priority = (osPriority_t)osPriorityNormal,
	};
	OS_THREAD_NEW_CSTM(TASK_BMI088_ReadTemp, bmi_temp_args, bmi_temp_attr, osWaitForever);
}

/* Start one save task per data stream (ACC, GYR, TEMP). */
static void spawn_save_data_tasks(osEventFlagsId_t save_data_done_ids[SAVE_TASK_COUNT]) {
	TASK_SaveData_ARGS save_data_args = {
		.data_topic = &data_topic_acc_ptr,
		.data_type = FLASH_DATA_TYPE_ACC,
		.done_flags = save_data_done_ids[0]
	};
	osThreadAttr_t save_data_attr = {
		.name = "TASK_SaveAccData",
		.priority = (osPriority_t)osPriorityNormal,
	};
	OS_THREAD_NEW_CSTM(TASK_SaveData, save_data_args, save_data_attr, osWaitForever);

	TASK_SaveData_ARGS save_gyr_data_args = {
		.data_topic = &data_topic_gyr_ptr,
		.data_type = FLASH_DATA_TYPE_GYR,
		.done_flags = save_data_done_ids[1]
	};
	osThreadAttr_t save_gyr_data_attr = {
		.name = "TASK_SaveGyrData",
		.priority = (osPriority_t)osPriorityNormal,
	};
	OS_THREAD_NEW_CSTM(TASK_SaveData, save_gyr_data_args, save_gyr_data_attr, osWaitForever);

	TASK_SaveData_ARGS save_temp_data_args = {
		.data_topic = &data_topic_temp_ptr,
		.data_type = FLASH_DATA_TYPE_TEMP,
		.done_flags = save_data_done_ids[2]
	};
	osThreadAttr_t save_temp_data_attr = {
		.name = "TASK_SaveTempData",
		.priority = (osPriority_t)osPriorityNormal,
	};
	OS_THREAD_NEW_CSTM(TASK_SaveData, save_temp_data_args, save_temp_data_attr, osWaitForever);
}

/* Wait for all save tasks to complete. */
static void wait_save_data_tasks(osEventFlagsId_t save_data_done_ids[SAVE_TASK_COUNT]) {
	osEventFlagsWait(save_data_done_ids[0], EVENT_FLAGS_DONE_BIT, osFlagsWaitAll, osWaitForever);
	osEventFlagsWait(save_data_done_ids[1], EVENT_FLAGS_DONE_BIT, osFlagsWaitAll, osWaitForever);
	osEventFlagsWait(save_data_done_ids[2], EVENT_FLAGS_DONE_BIT, osFlagsWaitAll, osWaitForever);
}

/* Read back flash and print decoded frames to USB CDC. */
static void print_flash_contents(void) {
	W25Q_STATE w25q_state = W25Q_PARAM_ERR;

	StaticEventGroup_t read_done_flags;
	osEventFlagsId_t read_done_flags_id = init_event_flags("ReadFlashDoneFlags", &read_done_flags);

	flash_data_t read_data;
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
	uint32_t current_addr = 0;

	while (!cdc_port_open) {
		osDelay(CDC_READY_POLL_DELAY_MS);
	}

	while (current_addr < flash_addr) {
		osEventFlagsClear(read_done_flags_id, EVENT_FLAGS_CLEAR_MASK);
		read_args.addr = current_addr;
		OS_THREAD_NEW_CSTM(TASK_W25Q_ReadData, read_args, read_attr, osWaitForever);
		osEventFlagsWait(read_done_flags_id, EVENT_FLAGS_DONE_BIT, osFlagsWaitAll, osWaitForever);

		switch (read_data.type) {
			case FLASH_DATA_TYPE_ACC:
				snprintf(usb_buffer, sizeof(usb_buffer), "ACC  t=%10lu X=%+10.3f  Y=%+10.3f  Z=%+10.3f  m/s2\r\n",
					read_data.data.acc.ts, read_data.data.acc.data.x, read_data.data.acc.data.y, read_data.data.acc.data.z);
				break;
			case FLASH_DATA_TYPE_GYR:
				snprintf(usb_buffer, sizeof(usb_buffer), "GYR  t=%10lu X=%+10.3f  Y=%+10.3f  Z=%+10.3f  deg/s\r\n",
					read_data.data.gyr.ts, read_data.data.gyr.data.x, read_data.data.gyr.data.y, read_data.data.gyr.data.z);
				break;
			case FLASH_DATA_TYPE_TEMP:
				snprintf(usb_buffer, sizeof(usb_buffer), "TEMP t=%10lu T=%+6.2f °C\r\n",
					read_data.data.temp.ts, read_data.data.temp.data);
				break;
			default:
				snprintf(usb_buffer, sizeof(usb_buffer), "Unknown data type at address 0x%08lX\r\n", current_addr);
				break;
		}

		CDC_Transmit_FS((uint8_t *)usb_buffer, strlen(usb_buffer));
		osDelay(CDC_TX_PACE_DELAY_MS);

		current_addr += sizeof(flash_data_t);
	}
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

	TASK_POOL_CREATE(TASK_InitProject);
	TASK_POOL_CREATE(TASK_SaveData);

	flash_addr_sem_id = osSemaphoreNew(1, 1, &(osSemaphoreAttr_t){
		.name = "FlashAddrSem",
		.cb_mem = &flash_addr_sem_cd,
		.cb_size = sizeof(flash_addr_sem_cd)
	});

	osThreadAttr_t main_loop_attr = {
		.name = "TASK_InitProject",
		.priority = (osPriority_t)osPriorityNormal,
	};
	TASK_InitProject_ARGS main_loop_args = {
		.dummy = NULL, // Placeholder to keep a non-empty args struct
	};
	OS_THREAD_NEW_CSTM(TASK_InitProject, main_loop_args, main_loop_attr, osWaitForever);

}

TASK_POOL_ALLOCATE(TASK_InitProject);
void TASK_InitProject(void *argument) {
	(void)argument;

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

	w25q_state = run_flash_erase_first_block();
	if (w25q_state != W25Q_OK) { osThreadExit_Cstm(); }

	// BMI init runs in parallel and is checked here.
	osEventFlagsWait(bmi_init_done_flags_id, EVENT_FLAGS_DONE_BIT, osFlagsWaitAll, osWaitForever);
	if (state != BMI_OK) { osThreadExit_Cstm(); }

	// ===================================================
	//                  Spawn measurement tasks
	// ===================================================

	spawn_measurement_tasks();

	// ===================================================
	//                  Spawn and wait save tasks
	// ===================================================

	StaticEventGroup_t save_data_done_flags[SAVE_TASK_COUNT];
	osEventFlagsId_t save_data_done_ids[SAVE_TASK_COUNT];
	for (int i = 0; i < (int)SAVE_TASK_COUNT; i++) {
		save_data_done_ids[i] = init_event_flags("SaveDataDoneFlags", &save_data_done_flags[i]);
	}

	spawn_save_data_tasks(save_data_done_ids);
	wait_save_data_tasks(save_data_done_ids);

	// =========================================================
	//                  Print flash content
	// =========================================================

	// Roughly ~100 frames per type -> about 6000 bytes total.

	print_flash_contents();

	osThreadExit_Cstm();
}


TASK_POOL_ALLOCATE(TASK_SaveData);
void TASK_SaveData(void *argument) {
	TASK_SaveData_ARGS *args = (TASK_SaveData_ARGS *)argument;
	if (!args->data_topic) {
		osThreadExit_Cstm();
	}

	flash_data_t flash_data = {
		.type = args->data_type,
	};

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

	uint32_t t0 = osKernelGetTickCount();

	// SAVE_DURATION_MS is in RTOS ticks (10 s at 1 kHz tick).
	while (osKernelGetTickCount() - t0 < SAVE_DURATION_MS) {
		data_sub_wait_for_data(&sub, osWaitForever);
		data_sub_read(&sub, &(flash_data.data.acc));

		osSemaphoreAcquire(flash_addr_sem_id, osWaitForever);

		w25q_write_args.addr = flash_addr;
		OS_THREAD_NEW_CSTM(TASK_W25Q_WriteData, w25q_write_args, w25q_write_attr, osWaitForever);
		osEventFlagsWait(w25q_done_id, EVENT_FLAGS_DONE_BIT, osFlagsWaitAll, osWaitForever);
		if (w25q_state != W25Q_OK) {
			osSemaphoreRelease(flash_addr_sem_id);
			osThreadExit_Cstm();
		}

		flash_addr += sizeof(flash_data);

		osSemaphoreRelease(flash_addr_sem_id);
	}

	if (args->done_flags) { osEventFlagsSet(args->done_flags, EVENT_FLAGS_DONE_BIT); }
	osThreadExit_Cstm();
}
