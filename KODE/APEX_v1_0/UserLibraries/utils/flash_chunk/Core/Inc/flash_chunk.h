#ifndef FLASH_CHUNK_H
#define FLASH_CHUNK_H

#include <stdint.h>
#include <stddef.h>

#include "main_config.h"

#if (APEX_CFG_SCHED_RTOS == 1)
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#endif

typedef struct flash_chunk_t {
    const char  *name;
    uint32_t     start_addr;
    uint32_t     size;
} flash_chunk_t;

typedef enum flash_chunk_ptr_type_t {
    FLASH_CHUNK_PTR_TYPE_BOUNDEDARY = 0,
    FLASH_CHUNK_PTR_TYPE_CIRCULAR
} flash_chunk_ptr_type_t;

typedef struct flash_chunk_ptr_t {
    flash_chunk_t           *chunk;
    uint32_t                offset;
    flash_chunk_ptr_type_t  type;

#if (APEX_CFG_SCHED_RTOS == 1)
    osSemaphoreId_t     sem;
    StaticSemaphore_t   sem_cd;
#endif

} flash_chunk_ptr_t;

typedef enum flash_chunk_status_t {
    FLASH_CHUNK_STATUS_OK = 0,
    FLASH_CHUNK_STATUS_START_REACHED,
    FLASH_CHUNK_STATUS_END_REACHED,
} flash_chunk_status_t;

void flash_chunk_init(flash_chunk_t *chunk, const char *name, uint32_t start_addr, uint32_t size);
void flash_chunk_ptr_init(flash_chunk_ptr_t *ptr, flash_chunk_t *chunk, flash_chunk_ptr_type_t type);
uint32_t flash_chunk_ptr_get_and_update(flash_chunk_ptr_t *ptr, int32_t offset, flash_chunk_status_t *status);

#endif /* FLASH_CHUNK_H */