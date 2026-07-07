#ifndef FLASH_STREAM_H
#define FLASH_STREAM_H


#include "stm32f4xx_hal.h"

#include "tools.h"
#include "w25q.h"


#define DATA_NUMBER             11 // 3 accel + 3 gyro + 1 temp + 1 pressure + 1 longitude + 1 latitude + 1 time
#define DATA_SIZE               DATA_NUMBER * sizeof(float)

#define LAST_SECTOR_ADDR        0xFFF000

#define SECTOR_SIZE             0x1000
#define INFO_SIZE               2*sizeof(uint32_t) + sizeof(uint8_t)

#define NUM_INFO                ((int)SECTOR_SIZE) / ((int)INFO_SIZE)
#define LAST_INFO_ADDR          LAST_SECTOR_ADDR + ((int)NUM_INFO - 1) * ((int)INFO_SIZE)


typedef struct FLASH_STREAM {
    W25Q_t      *flash_chip;

    uint32_t     write_ptr;
    uint32_t     read_ptr;

    size_t       data_size;

    uint32_t     last_info_ptr;
} FLASH_STREAM;


void flash_stream_init(FLASH_STREAM* stream, W25Q_t* flash_chip, size_t data_size);

void flash_stream_write(FLASH_STREAM* stream, uint8_t* data, uint16_t len);
void flash_stream_read(FLASH_STREAM* stream, uint8_t* data, uint16_t len);


#endif      // FLASH_STREAM_H
