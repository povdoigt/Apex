#include "utils/gms.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "usbd_cdc_if.h"


GMS GMS_memory;


#define GMS_PTR_TO_BLOCK(ptr) \
    (__GMS_ATOMIC_BLOCK*)((uintptr_t)ptr - (uintptr_t)offsetof(__GMS_ATOMIC_BLOCK, data))
#define GMS_BLOCK_TO_PTR(block) \
    (void*)(&(block->data))


uintptr_t GMS_last_possible_ptr(GMS *memory);
void *GMS_next_block(GMS *memory, __GMS_ATOMIC_BLOCK *block);
void *GMS_prev_block(GMS *memory, __GMS_ATOMIC_BLOCK *block);
size_t get_block_idx(GMS *memory, __GMS_ATOMIC_BLOCK *block);
void GMS_sprintf_block_idx(char *str, GMS *memory, __GMS_ATOMIC_BLOCK *block);
void GMS_print_debug(GMS *memory);


uintptr_t GMS_last_possible_ptr(GMS *memory) {
    return (uintptr_t)memory + GMS_MEM_SIZE - sizeof(__GMS_ATOMIC_BLOCK);
}

void *GMS_next_block(GMS *memory, __GMS_ATOMIC_BLOCK *block) {
    uintptr_t p = (uintptr_t)block + block->size + sizeof(__GMS_ATOMIC_BLOCK);
    if (p > GMS_last_possible_ptr(memory)) {
        return NULL;
    }
    return (void*)p;
}

void *GMS_prev_block(GMS *memory, __GMS_ATOMIC_BLOCK *block) {
    uintptr_t p = (uintptr_t)block - block->direct_prev_size - sizeof(__GMS_ATOMIC_BLOCK);
    if (p < (uintptr_t)memory) {
        return NULL;
    }
    return (void*)p;
}


void GMS_init(GMS *memory) {
    memory->free_blocks = (__GMS_ATOMIC_BLOCK*)memory->blocks;

    memory->free_blocks->is_free = true;
    memory->free_blocks->free_prev = NULL;
    memory->free_blocks->free_next = NULL;
    memory->free_blocks->direct_prev_size = 0;
    memory->free_blocks->size = GMS_MEM_SIZE - sizeof(__GMS_ATOMIC_BLOCK);

    #if GMS_DEBUG_STORE
    memory->debug_store[0] = '\0';
    #endif

    GMS_print_debug(memory);
}
static uint32_t gms_max_size;

void *GMS_alloc(GMS *memory, size_t size) {
    __GMS_ATOMIC_BLOCK *block = memory->free_blocks;
    
    if (size > 256) {
        __NOP();
    }
    gms_max_size = size > gms_max_size ? size : gms_max_size;


    while (block && block->size < size) {
        block = block->free_next;
    }

    if (!block) {
        GMS_print_debug(memory);
        Error_Handler();
        return NULL;
    }

    size_t avalable_size = block->size;

    __GMS_ATOMIC_BLOCK *old_direct_next = GMS_next_block(memory, block);

    block->size = size;
    block->is_free = false;

    __GMS_ATOMIC_BLOCK *new_direct_next = GMS_next_block(memory, block);

    // If there is enough space to create a new block
    //  - If there is enough space anyway AND
    //  - If block is the last block OR
    //  - If block is not the last block AND there is enough space between the two blocks
    if (((new_direct_next != NULL) &&
        (old_direct_next == NULL)) ||
        (old_direct_next != NULL && (uintptr_t)old_direct_next - (uintptr_t)new_direct_next > sizeof(__GMS_ATOMIC_BLOCK))) {
            new_direct_next->is_free = true;
            new_direct_next->free_prev = NULL;
            new_direct_next->free_next = NULL;
            new_direct_next->direct_prev_size = size;
            new_direct_next->size = avalable_size - size - sizeof(__GMS_ATOMIC_BLOCK);

        if (old_direct_next) {
            new_direct_next->free_next = block->free_next;
            ((__GMS_ATOMIC_BLOCK *)(block->free_next))->free_prev = new_direct_next;
            ((__GMS_ATOMIC_BLOCK *)GMS_next_block(memory, new_direct_next))->direct_prev_size = new_direct_next->size;
        }
            block->free_next = new_direct_next;
    } else {
        block->size = avalable_size;
    }

    if (block->free_prev) {
        ((__GMS_ATOMIC_BLOCK *)(block->free_prev))->free_next = block->free_next;
        ((__GMS_ATOMIC_BLOCK *)(block->free_next))->free_prev = block->free_prev;
    } else {
        memory->free_blocks = block->free_next;
        memory->free_blocks->free_prev = NULL;
    }
    block->free_prev = NULL;
    block->free_next = NULL;
    
    GMS_print_debug(memory);

    if(block->data < 1) {
       Error_Handler();
    }

    return block->data;
}

void GMS_free(GMS *memory, void *ptr) {
    __GMS_ATOMIC_BLOCK *block = GMS_PTR_TO_BLOCK(ptr);
    __GMS_ATOMIC_BLOCK *direct_prev = GMS_prev_block(memory, block);
    __GMS_ATOMIC_BLOCK *direct_next = GMS_next_block(memory, block);

    // Check if can merge with previous block
    if (direct_prev && direct_prev->is_free) {
        direct_prev->size += sizeof(__GMS_ATOMIC_BLOCK) + block->size;
        if (direct_prev->free_prev) {
            __GMS_ATOMIC_BLOCK *direct_prev_free_prev = (__GMS_ATOMIC_BLOCK *)(direct_prev->free_prev);
            __GMS_ATOMIC_BLOCK *direct_prev_free_next = (__GMS_ATOMIC_BLOCK *)(direct_prev->free_next);
            if (direct_prev_free_prev) {
                direct_prev_free_prev->free_next = direct_prev_free_next;
            }
            if (direct_prev == memory->free_blocks) {
                memory->free_blocks = direct_prev_free_next;
            }
            if (direct_prev_free_next) {
                direct_prev_free_next->free_prev = direct_prev_free_prev;
            }
        }
        block = direct_prev;
    }

    // Check if can merge with next block
    if (direct_next) {
        if (direct_next->is_free) {
            block->size += sizeof(__GMS_ATOMIC_BLOCK) + direct_next->size;
            __GMS_ATOMIC_BLOCK *direct_next_free_prev = (__GMS_ATOMIC_BLOCK *)(direct_next->free_prev);
            __GMS_ATOMIC_BLOCK *direct_next_free_next = (__GMS_ATOMIC_BLOCK *)(direct_next->free_next);
            if (direct_next_free_prev) {
                direct_next_free_prev->free_next = direct_next_free_next;
            }
            if (direct_next == memory->free_blocks) {
                memory->free_blocks = direct_next_free_next;
            }
            if (direct_next_free_next) {
                direct_next_free_next->free_prev = direct_next_free_prev;
            }
        }
    }

    // Update direct_next
    direct_next = GMS_next_block(memory, block);
    if (direct_next) {
        direct_next->direct_prev_size = block->size;
    }

    // Add block to free list
    block->is_free = true;
    block->free_next = block == memory->free_blocks ? NULL : memory->free_blocks;
    block->free_prev = NULL;
    memory->free_blocks = block;


    // Link block to previous free block
    if (block->free_next) {
        ((__GMS_ATOMIC_BLOCK *)(block->free_next))->free_prev = block;
    }

    GMS_print_debug(memory);
}


size_t get_block_idx(GMS *memory, __GMS_ATOMIC_BLOCK *block) {
    __GMS_ATOMIC_BLOCK *current = (__GMS_ATOMIC_BLOCK*)memory->blocks;

    size_t idx = 0;
    while (current && current != block) {
        current = GMS_next_block(memory, current);
        idx++;
    }
    if (current != block) {
        return (size_t)-1;
    }
    return idx;
}

void GMS_sprintf_block_idx(char *str, GMS *memory, __GMS_ATOMIC_BLOCK *block) {
    if (!block) {
        sprintf(str, "NULL");
        return;
    }
    size_t idx = get_block_idx(memory, block);
    if (idx == (size_t)-1) {
        sprintf(str, "block ???");
    } else {
        sprintf(str, "block %lu", (long unsigned int)idx);
    }
}
size_t GMS_get_free_size(GMS *memory) {
    size_t total_free = 0;
    __GMS_ATOMIC_BLOCK *block = memory->free_blocks;

    while (block) {
        total_free += block->size;
        block = block->free_next;
    }

    return total_free;
}

#if GMS_DEBUG_PRINT_USB
void GMS_print_debug(GMS *memory) {
    __GMS_ATOMIC_BLOCK *block = (__GMS_ATOMIC_BLOCK*)(memory->blocks);

    size_t i = 0;

    char str_free[10], str_prev[10], str_next[10], buff0[256], buff1[256];    

    GMS_sprintf_block_idx(str_free, memory, memory->free_blocks);

    do {
        buff1[0] = '\0';

        GMS_sprintf_block_idx(str_prev, memory, block->free_prev);
        GMS_sprintf_block_idx(str_next, memory, block->free_next);

        sprintf(buff0, "Block %d: %p\n", i, (void *)block);
        strcat(buff1, buff0);
        sprintf(buff0, "  is_free: %d\n", block->is_free);
        strcat(buff1, buff0);
        sprintf(buff0, "  free_prev: %p %-10s\n", block->free_prev, str_prev);
        strcat(buff1, buff0);
        sprintf(buff0, "  free_next: %p %-10s\n", block->free_next, str_next);
        strcat(buff1, buff0);
        sprintf(buff0, "  direct_prev_size: %lu\n", (long unsigned int)block->direct_prev_size);
        strcat(buff1, buff0);
        sprintf(buff0, "  size: %lu\n", (long unsigned int)block->size);
        strcat(buff1, buff0);

        HAL_Delay(10);
        CDC_Transmit_FS((uint8_t*)buff1, strlen(buff1));

        i++;
        block = GMS_next_block(memory, block);
    } while (block);

    sprintf(buff1, "Free blocks: %p %-10s\n\n", (void *)memory->free_blocks, str_free);
    HAL_Delay(10);
    CDC_Transmit_FS((uint8_t*)buff1, strlen(buff1));
}
#else
#if GMS_DEBUG_STORE
void GMS_print_debug(GMS *memory) {
    __GMS_ATOMIC_BLOCK *block = (__GMS_ATOMIC_BLOCK*)(memory->blocks);

    size_t i = 0;

    char str_free[10], str_prev[10], str_next[10], buff0[256], buff1[256];    

    GMS_sprintf_block_idx(str_free, memory, memory->free_blocks);

    do {
        buff1[0] = '\0';

        GMS_sprintf_block_idx(str_prev, memory, block->free_prev);
        GMS_sprintf_block_idx(str_next, memory, block->free_next);

        sprintf(buff0, "Block %d: %p\n", i, (void *)block);
        strcat(buff1, buff0);
        sprintf(buff0, "  is_free: %d\n", block->is_free);
        strcat(buff1, buff0);
        sprintf(buff0, "  free_prev: %p %-10s\n", block->free_prev, str_prev);
        strcat(buff1, buff0);
        sprintf(buff0, "  free_next: %p %-10s\n", block->free_next, str_next);
        strcat(buff1, buff0);
        sprintf(buff0, "  direct_prev_size: %lu\n", (long unsigned int)block->direct_prev_size);
        strcat(buff1, buff0);
        sprintf(buff0, "  size: %lu\n", (long unsigned int)block->size);
        strcat(buff1, buff0);

        strcat(memory->debug_store, buff1);

        i++;
        block = GMS_next_block(memory, block);
    } while (block);

    sprintf(buff1, "Free blocks: %p %-10s\n\n", (void *)memory->free_blocks, str_free);
    strcat(memory->debug_store, buff1);
}
#else
void GMS_print_debug(GMS *memory) {
    UNUSED(memory);
}
#endif
#endif