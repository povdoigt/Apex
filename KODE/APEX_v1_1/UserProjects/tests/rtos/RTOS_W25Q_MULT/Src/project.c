#include "project.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "drivers_config.h"

#include "scheduler.h"
#include "w25q.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

static uint8_t buffer[4096];

void setup(void) {

    TASK_POOL_CREATE(TASK_W25Q_Init);
    TASK_POOL_CREATE(TASK_W25Q_SendCmdAddr);
    TASK_POOL_CREATE(TASK_W25Q_WriteData);

    TASK_POOL_CREATE(TASK_Main);

    memset(buffer, 0x5a, sizeof(buffer));

    osThreadAttr_t main_attr = {
		.name = "TASK_Main",
		.priority = (osPriority_t)osPriorityNormal,
	};
	OS_THREAD_NEW_CSTM(TASK_Main, (TASK_Main_ARGS){}, main_attr, osWaitForever);

}

TASK_POOL_ALLOCATE(TASK_Main);
void TASK_Main(void *argument) {
    
    StaticEventGroup_t done_flags_mem;
    osEventFlagsId_t done_flags_id = osEventFlagsNew(&((osEventFlagsAttr_t){
        .name = "W25Q_Init_Done_Flags",
        .cb_mem = &done_flags_mem,
        .cb_size = sizeof(done_flags_mem)
    }));
    osThreadAttr_t w25q_init_attr = {
        .name = "TASK_W25Q_Init",
        .priority = (osPriority_t)osPriorityNormal,
    };
    W25Q_STATE w25q_state;
    TASK_W25Q_Init_ARGS w25q_init_args = {
        .chip = &w25q,
        .hspi = &hspi2,
        .cs_bank = CS_FLASH_GPIO_Port,
        .cs_pin = CS_FLASH_Pin,
        .result = &w25q_state,
        .done_flags = done_flags_id
    };
    OS_THREAD_NEW_CSTM(TASK_W25Q_Init, w25q_init_args, w25q_init_attr, osWaitForever);

    // Wait for W25Q init to complete
    osEventFlagsWait(done_flags_id, 1, osFlagsWaitAll, osWaitForever);
    if (w25q_state != W25Q_OK) {
        osThreadExit_Cstm();
    }

    // Erase first flash block
    TASK_W25Q_SendCmdAddr_ARGS erase_args = {
        .chip = &w25q,
        .cmd = W25Q_64KB_BLOCK_ERASE_4B,
        .addr = 0,
        .result = &w25q_state,
        .done_flags = done_flags_id
    };
    osThreadAttr_t erase_attr = {
        .name = "TASK_W25Q_EraseSector",
        .priority = (osPriority_t)osPriorityNormal,
    };
    OS_THREAD_NEW_CSTM(TASK_W25Q_SendCmdAddr, erase_args, erase_attr, osWaitForever);

    // Wait for erase to complete
    osEventFlagsWait(done_flags_id, 1, osFlagsWaitAll, osWaitForever);
    if (w25q_state != W25Q_OK) {    
        osThreadExit_Cstm();
    }

    if (W25Q_WaitForReady_RTOS_NoLock(&w25q) != W25Q_OK) {
        osThreadExit_Cstm();
    }


    // Spawn 5 concurrent write tasks to test semaphore locking and concurrent access to the flash chip.
    
    StaticEventGroup_t write_done_flags_mem[5];
    osEventFlagsId_t write_done_flags[5];
    for (int i = 0; i < 5; i++) {
        write_done_flags[i] = osEventFlagsNew(&((osEventFlagsAttr_t){
            .name = "W25Q_Write_Done_Flags",
            .cb_mem = &write_done_flags_mem[i],
            .cb_size = sizeof(write_done_flags_mem[i])
        }));
    }
    W25Q_STATE write_states[5];

    for (int i = 0; i < 5; i++) {
        TASK_W25Q_WriteData_ARGS write_args = {
            .chip = &w25q,
            .buffer = buffer,
            .addr = i * 4096, // write to different addresses to avoid overwriting
            .buf_size = sizeof(buffer),
            .result = &write_states[i],
            .done_flags = write_done_flags[i]
        };

        osThreadAttr_t write_attr = {
            .name = "TASK_W25Q_WriteData",
            .priority = (osPriority_t)osPriorityNormal,
        };

        OS_THREAD_NEW_CSTM(TASK_W25Q_WriteData, write_args, write_attr, osWaitForever);
    }

    // Wait for all writes to complete and check results
    for (int i = 0; i < 5; i++) {
        osEventFlagsWait(write_done_flags[i], 1, osFlagsWaitAll, osWaitForever);
        if (write_states[i] != W25Q_OK) {
            osThreadExit_Cstm();
        }
    }

    __NOP();

}
