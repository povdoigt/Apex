#include "utils/flash_stream.h"
#include <stdlib.h>


void flash_stream_init(FLASH_STREAM* stream, W25Q_Chip* flash_chip, size_t data_size) {
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

void flash_stream_write_float(FLASH_STREAM* stream, float data) {
    FLOAT_U32_UNION data_float_u32 = { .float_type = data };
    uint8_t flash_buffer[4] = {data_float_u32.uint32_type >> 24,
                               data_float_u32.uint32_type >> 16,
                               data_float_u32.uint32_type >> 8,
                               data_float_u32.uint32_type >> 0 };
    flash_stream_write(stream, flash_buffer, 4);
}

void flash_stream_read_float(FLASH_STREAM* stream, float* data) {
    uint8_t flash_buffer[4];
    FLOAT_U32_UNION data_float_u32;
    flash_stream_read(stream, flash_buffer, 4);
    data_float_u32.uint32_type = (flash_buffer[0] << 24 |
                                  flash_buffer[1] << 16 |
                                  flash_buffer[2] << 8 |
                                  flash_buffer[3] << 0);
    *data = data_float_u32.float_type;
}

void flash_stream_write_floats(FLASH_STREAM* stream, float* data, uint16_t len) {
    uint8_t flash_buffer[256];

    for (int i = 0; i < len; i++) {
        FLOAT_U32_UNION data_float_u32 = { .float_type = data[i] };
        flash_buffer[i * 4 + 0] = data_float_u32.uint32_type >> 24;
        flash_buffer[i * 4 + 1] = data_float_u32.uint32_type >> 16;
        flash_buffer[i * 4 + 2] = data_float_u32.uint32_type >> 8;
        flash_buffer[i * 4 + 3] = data_float_u32.uint32_type >> 0;
    }

    flash_stream_write(stream, flash_buffer, len * 4);
}

void flash_stream_read_floats(FLASH_STREAM* stream, float* data, uint16_t len) {
    uint8_t flash_buffer[256];

    flash_stream_read(stream, flash_buffer, len * 4);

    for (int i = 0; i < len; i++) {
        FLOAT_U32_UNION data_float_u32;
        data_float_u32.uint32_type = (flash_buffer[i * 4 + 0] << 24 |
                                      flash_buffer[i * 4 + 1] << 16 |
                                      flash_buffer[i * 4 + 2] << 8 |
                                      flash_buffer[i * 4 + 3] << 0);
        data[i] = data_float_u32.float_type;
    }
}


// TASK_POOL_CREATE(ASYNC_fs_read_write);

// void ASYNC_fs_read_write_init(TASK *self,
//                               FLASH_STREAM *stream, uint8_t *data,
//                               size_t data_size) {
//     ASYNC_fs_read_write_CONTEXT *context = (ASYNC_fs_read_write_CONTEXT*)self->context;

//     context->stream = stream;
//     context->data = data;
//     context->data_size = data_size;
// }

// TASK_RETURN ASYNC_fs_write(SCHEDULER *scheduler, TASK *self) {
//     ASYNC_fs_read_write_CONTEXT *context = (ASYNC_fs_read_write_CONTEXT*)self->context;

//     TASK *task = SCHEDULER_add_task(scheduler, ASYNC_W25Q_WriteData, false, (OBJ_POOL *)ASYNC_W25Q_WriteData_POOL);
//     ASYNC_W25Q_WriteData_init(task,
//                               context->stream->flash_chip,
//                               context->data,
//                               context->data_size,
//                               context->stream->write_ptr);
//     task->is_done = self->is_done;
//     self->is_done = NULL;

//     context->stream->write_ptr += context->data_size;

//     return TASK_RETURN_STOP;
// }

// TASK_RETURN ASYNC_fs_read(SCHEDULER *scheduler, TASK *self) {
//     ASYNC_fs_read_write_CONTEXT *context = (ASYNC_fs_read_write_CONTEXT*)self->context;

//     TASK *task = SCHEDULER_add_task(scheduler, ASYNC_W25Q_ReadData, false, (OBJ_POOL *)ASYNC_W25Q_ReadData_POOL);
//     ASYNC_W25Q_ReadData_init(task,
//                              context->stream->flash_chip,
//                              context->data,
//                              context->data_size,
//                              context->stream->read_ptr);
//     task->is_done = self->is_done;
//     self->is_done = NULL;

//     context->stream->read_ptr += context->data_size;

//     return TASK_RETURN_STOP;
// }
