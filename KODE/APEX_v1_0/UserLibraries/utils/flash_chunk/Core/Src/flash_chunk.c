#include "flash_chunk.h"

void flash_chunk_init(flash_chunk_t *chunk, const char *name, uint32_t start_addr, uint32_t size) {
    chunk->name = name;
    chunk->start_addr = start_addr;
    chunk->size = size;
}

void flash_chunk_ptr_init(flash_chunk_ptr_t *ptr, flash_chunk_t *chunk, flash_chunk_ptr_type_t type) {
    ptr->chunk = chunk;
    ptr->offset = 0;
    ptr->type = type;

#if (APEX_CFG_SCHED_RTOS == 1)
    osSemaphoreAttr_t sem_attr = {
        .name = "FlashChunkPtrSem",
        .cb_mem = &(ptr->sem_cd),
        .cb_size = sizeof(ptr->sem_cd)
    };
    ptr->sem = osSemaphoreNew(1, 1, &sem_attr);
#endif
}

uint32_t flash_chunk_ptr_get_and_update(flash_chunk_ptr_t *ptr, int32_t offset, flash_chunk_status_t *status) {
    uint32_t new_offset = ptr->offset + offset;
    flash_chunk_status_t st;

#if (APEX_CFG_SCHED_RTOS == 1)
    osSemaphoreAcquire(ptr->sem, osWaitForever);
#endif

    switch (ptr->type) {
        case FLASH_CHUNK_PTR_TYPE_BOUNDEDARY: {
            if (new_offset >= ptr->chunk->size) {
                // Stay at the end of the chunk minus the offset that exceeded it
                // This waywe can't write outside the chunk
                new_offset = ptr->chunk->size - offset;
                st = FLASH_CHUNK_STATUS_END_REACHED;
            } else if (new_offset < 0) {
                new_offset = 0;
                st = FLASH_CHUNK_STATUS_START_REACHED;
            }
            break;
        }
        case FLASH_CHUNK_PTR_TYPE_CIRCULAR: {
            if (new_offset >= ptr->chunk->size) {
                new_offset = new_offset % ptr->chunk->size;
                st = FLASH_CHUNK_STATUS_END_REACHED;
            } else if (new_offset < 0) {
                new_offset = (ptr->chunk->size + (new_offset % ptr->chunk->size)) % ptr->chunk->size;
                st = FLASH_CHUNK_STATUS_START_REACHED;
            }
            break;
        }
    }
    uint32_t old_offset = ptr->offset;
    ptr->offset = new_offset;

    if (status) {
        *status = st;
    }

#if (APEX_CFG_SCHED_RTOS == 1)
    osSemaphoreRelease(ptr->sem);
#endif

    return ptr->chunk->start_addr + old_offset;
}
