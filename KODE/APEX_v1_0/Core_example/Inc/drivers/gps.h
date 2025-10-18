/*
 * gps.h
 *
 *  Created on: Nov 15, 2019
 *      Author: Bulanov Konstantin
 */

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include "peripherals/usart.h"

#ifndef GPS_H_
#define GPS_H_

#define GPS_DEBUG             0
#define GPS_BUF_SIZE        128                             // GPS parser buffer size
#define GPS_MAX_SENTENCE     20                             // Max number of sentences in GPS message
// #define RX_BUF_SIZE         GPS_MAX_SENTENCE*GPS_BUF_SIZE   // UART receive buffer size
#define RX_BUF_SIZE         512                             // UART receive buffer size

typedef struct {
    UART_HandleTypeDef *uart;

    uint8_t rx_buffer[RX_BUF_SIZE]; // buffer for received data

    // calculated values
    float dec_longitude;
    float dec_latitude;
    float altitude_ft;

    // GGA - Global Positioning System Fixed Data
    float nmea_longitude;
    float nmea_latitude;
    float utc_time;
    char ns, ew;
    int lock;
    int satelites;
    float hdop;
    float msl_altitude;
    char msl_units;

    // RMC - Recommended Minimmum Specific GNS Data
    char rmc_status;
    float speed_k;
    float course_d;
    int date;

    // GLL
    char gll_status;
    bool has_fix_once;

    // VTG - Course over ground, ground speed
    float course_t; // ground speed true
    char course_t_unit;
    float course_m; // magnetic
    char course_m_unit;
    char speed_k_unit;
    float speed_km; // speek km/hr
    char speed_km_unit;
} GPS_t;




// #if (GPS_DEBUG == 1)
// void GPS_print(char *data);
// #endif

void    GPS_Init(GPS_t *GPS, UART_HandleTypeDef *uart);
void    GPS_UART_Receive(GPS_t *gps);
void    GPS_USBPrint(GPS_t *gps);
// void    GPS_UART_CallBack(UART_CALLBACK_PARAM_PTR param, UART_HandleTypeDef *huart, uint16_t size);
int     GPS_validate(char *nmeastr);
void    GPS_parse(GPS_t *GPS, char *GPSstrParse);
void    GPS_parse_GLL(GPS_t *GPS, char **elems);
void    GPS_parse_GGA(GPS_t *GPS, char **elems);
void    GPS_parse_VTG(GPS_t *GPS, char **elems);
float   GPS_nmea_to_dec(float deg_coord, char nsew);



// ==================================================


#define ASYNC_GPS_Rx_DMA_NUMBER 10


typedef struct ASYNC_GPS_Rx_DMA_CONTEXT {
    GPS_t *gps;

    uint8_t *buf;
    uint16_t size;
} ASYNC_GPS_Rx_DMA_CONTEXT;


// ==================================================

#define ASYNC_UART_RxToIdle_DMA_init_GPS(task, gps) \
    ASYNC_UART_RxToIdle_DMA_init(task, gps->uart, gps->rx_buffer, RX_BUF_SIZE)








#endif /* GPS_H_ */