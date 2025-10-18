#ifndef GMS_H
#define GMS_H

#include <stdbool.h>
#include "stm32f4xx_hal.h"


#define GMS_MEM_SIZE 256 * 64

#define GMS_DEBUG_PRINT_USB 0

#define GMS_DEBUG_STORE 0


typedef struct __GMS_ATOMIC_BLOCK {
    void    *free_prev;
    void    *free_next;
    size_t   direct_prev_size;
    size_t   size;
    bool     is_free;
    uint8_t  data[];
} __GMS_ATOMIC_BLOCK;
// sizeof(__GMS_ATOMIC_BLOCK) = (64 machine) 40 + len(data); (32 machine) 20 + len(data)

typedef struct GMS {
    uint8_t              blocks[GMS_MEM_SIZE];
    __GMS_ATOMIC_BLOCK  *free_blocks;
#if GMS_DEBUG_STORE
    char                 debug_store[4096];
#endif
} GMS;


extern GMS GMS_memory;


void GMS_init(GMS *memory);

void *GMS_alloc(GMS *memory, size_t size);

void GMS_free(GMS *memory, void *block);

size_t GMS_get_free_size(GMS *memory);

#endif // GMS_H
