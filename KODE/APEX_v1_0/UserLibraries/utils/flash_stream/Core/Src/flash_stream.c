#include "flash_stream.h"
#include <stdlib.h>


void flash_stream_init(FLASH_STREAM* stream, W25Q_t* flash_chip, size_t data_size) {
    stream->flash_chip = flash_chip;
    stream->write_ptr = 0;
    stream->read_ptr = 0;

    stream->data_size = data_size;

    // flash_stream_find_last_info_ptr(stream);
}


void flash_stream_write(FLASH_STREAM* stream, uint8_t* data, uint16_t len) {
    if (stream->write_ptr + stream->data_size < LAST_SECTOR_ADDR) {
        W25Q_WriteData(stream->flash_chip, data, stream->write_ptr, len);
        stream->write_ptr += len;
    }
}

void flash_stream_read(FLASH_STREAM* stream, uint8_t* data, uint16_t len) {
    W25Q_ReadData(stream->flash_chip, data, stream->read_ptr, len);
    stream->read_ptr += len;
}
