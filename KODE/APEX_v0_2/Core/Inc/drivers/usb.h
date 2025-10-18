#ifndef USB_H
#define USB_H

#include "stm32f4xx_hal.h"

#include "utils/scheduler.h"
#include "utils/circular_buffer.h"


#define USB_BEGIN_PACKET_0 0xa5     // Magic number for USB packet start
#define USB_BEGIN_PACKET_1 0x5a     // Magic number for USB packet start


typedef struct USB_PACKET {
    uint8_t begin[2]; // 0xa5, 0x5a
    uint8_t type; // Type of packet
    uint16_t length; // Length of data
    uint8_t *data; // Data payload
    uint16_t checksum; // Checksum for data integrity
} USB_PACKET;

typedef struct USB_PACKET_DATA {
    uint16_t sequence_num;
    uint8_t data[2048];
} USB_PACKET_DATA;


typedef enum {
    USB_PACKET_TYPE_DATA = 0x01, // Data packet
    USB_PACKET_TYPE_ACK = 0x02, // Acknowledgment packet
    USB_PACKET_TYPE_NACK = 0x03, // Negative acknowledgment packet
    USB_PACKET_TYPE_ERROR = 0x04, // Error packet
    USB_PACKET_TYPE_REQUEST_MEMORY = 0x05, // Request memory packet
} USB_PACKET_TYPE;


typedef struct LAST_USB_PACKET {
    USB_PACKET packet; // Last received USB packet
    uint8_t has_been_read; // Flag to indicate if the packet has been read
} LAST_USB_PACKET;


extern CIRCULAR_BUFFER usb_packet_buffer; // Circular buffer for USB packets



void USB_PACKET_init(USB_PACKET *packet, USB_PACKET_TYPE type, uint8_t *data, uint16_t length);

void USB_receive_data(uint8_t *buf, size_t size);




#endif // USB_H