#include "drivers/usb.h"
#include <string.h>


// uint8_t usb_packet_buffer[sizeof(USB_PACKET) * 10]; // Buffer for USB packets
// CIRCULAR_BUFFER usb_packet_buffer_cb = {
//     .buffer = usb_packet_buffer,
//     .head = 0,
//     .tail = 0,
//     .capacity = 10, // Capacity in elements
//     .max_size = sizeof(USB_PACKET) * 10, // Maximum size in bytes
//     .data_size = sizeof(USB_PACKET) // Size of each data element
// };

// uint8_t usb_data_buffer[2048]; // Buffer for USB data packets
// CIRCULAR_BUFFER usb_data_buffer_cb = {
//     .buffer = usb_data_buffer,
//     .head = 0,
//     .tail = 0,
//     .capacity = 2048, // Capacity in elements
//     .max_size = 2048, // Maximum size in bytes
//     .data_size = 1 // Size of each data element
// };


// void USB_receive_data(uint8_t *buf, size_t size) {
//     for (size_t i = 0; i < size; i++) {
//         CIRCULAR_BUFFER_one_byte_push(&usb_data_buffer_cb, buf[i]);
//     }
// }


// void USB_PACKET_init(USB_PACKET *packet, USB_PACKET_TYPE type, uint8_t *data, uint16_t length) {
//     packet->begin[0] = USB_BEGIN_PACKET_0;
//     packet->begin[1] = USB_BEGIN_PACKET_1;
//     packet->type = type;
//     packet->length = length;
//     packet->data = data;
    
//     // Calculate checksum
//     uint16_t checksum = 0;
//     for (uint16_t i = 0; i < length; i++) {
//         checksum += data[i];
//     }
//     packet->checksum = checksum;
// }


