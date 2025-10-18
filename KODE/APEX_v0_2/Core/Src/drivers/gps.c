/*
 * gps.c
 *
 *  Created on: Nov 15, 2019
 *      Author: Bulanov Konstantin
 *
 *  Contact information
 *  -------------------
 *
 * e-mail   :  leech001@gmail.com
 */

/*
 * |---------------------------------------------------------------------------------
 * | Copyright (C) Bulanov Konstantin,2019
 * |
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * |
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |---------------------------------------------------------------------------------
 */


#include <stdio.h>
#include <string.h>
#include "drivers/gps.h"
#include "utils/tools.h"
#include <usbd_cdc_if.h>



void GPS_Init(GPS_t *gps, UART_HandleTypeDef *uart) {
    gps->uart = uart;

    UART_Callback_struct_set(gps->uart, GPS_UART_CallBack, (UART_CALLBACK_PARAM_PTR)gps);

    // Set gps cfg to CFG-NAVSPG-DYNMODEL 8 (Airborne <4g)
    // uint8_t tx_buffer[17] = {
    //     0xB5,
    //     0x62,
    //     0x06,
    //     0x8A,
    //     0x09,
    //     0x00,
    //     0x00,
    //     0x01,
    //     0x00,
    //     0x00,
    //     0x21,
    //     0x00,
    //     0x11,
    //     0x20,
    //     0x00,
    //     0xEC,
    //     0x49
    // };
    // HAL_UART_Transmit(gps->uart, tx_buffer, 17, 100);
    // uint8_t tx[256];

    // tx[0]  = 0xB5;
    // tx[1]  = 0x62;
    // tx[2]  = 0x06;
    // tx[3]  = 0x8A;
    // tx[4]  = 0x0C;
    // tx[5]  = 0x00;
    // tx[6]  = 0x00;
    // tx[7]  = 0x02;
    // tx[8]  = 0x00;
    // tx[9]  = 0x00;
    // tx[10] = 0x01;
    // tx[11] = 0x00;
    // tx[12] = 0x52;
    // tx[13] = 0x40;
    // tx[14] = 0x00;
    // tx[15] = 0xC2;
    // tx[16] = 0x01;
    // tx[17] = 0x00;
    // tx[18] = 0xF6;
    // tx[19] = 0xC6;
    // HAL_UART_Transmit(gps->uart, tx, 20, 100);

    // uint8_t tx_buffer[8] = {
    //     0xB5,
    //     0x62,
    //     0x0A,
    //     0x04,
    //     0x00,
    //     0x00,
    //     0x0E,
    //     0x34,
    // };
    // HAL_UART_Transmit(gps->uart, tx_buffer, 8, 100);

    // uint8_t rx_buf[4096] = {0};
    // HAL_UART_Transmit(gps->uart, tx_buf, sizeof(tx_buf), 100);

    // gps->uart->Init.BaudRate = 115200; // Set baud rate to 115200
    // HAL_StatusTypeDef ret = HAL_UART_Init(gps->uart);
    // HAL_StatusTypeDef exp = HAL_OK;

    // HAL_Delay(1); // Wait for the GPS to initialize

    // HAL_UART_Receive(gps->uart, rx_buf, sizeof(rx_buf), 100);
    // HAL_Delay(1000);
    // CDC_Transmit_FS(rx_buf, sizeof(rx_buf));

    // HAL_UARTEx_ReceiveToIdle_DMA(gps->uart, gps->rx_buffer, RX_BUF_SIZE);

    __NOP();
}

// void GPS_UART_Receive(GPS_t *gps) {
//     HAL_UARTEx_ReceiveToIdle_DMA(gps->uart, gps->rx_buffer, RX_BUF_SIZE);
//     // __HAL_DMA_DISABLE_IT(gps_uart_cb->gps->uart->hdmarx, DMA_IT_HT);
// }


// void GPS_UART_CallBack(GPS_t *GPS, UART_HandleTypeDef *gps_uart) {
// 	if (rx_data != '\n' && rx_index < sizeof(rx_buffer)) {
// 		rx_buffer[rx_index++] = rx_data;
// 	} else {

// 		#if (GPS_DEBUG == 1)
// 		GPS_print((char*)rx_buffer);
// 		#endif

// 		if(GPS_validate((char*) rx_buffer))
// 			GPS_parse(GPS, (char*) rx_buffer);
// 		rx_index = 0;
// 		memset(rx_buffer, 0, sizeof(rx_buffer));
// 	}
// 	HAL_UART_Receive_IT(gps_uart, &rx_data, 1);
// }




void GPS_UART_CallBack(UART_CALLBACK_PARAM_PTR param, UART_HandleTypeDef *uart, uint16_t size) {
    GPS_t *gps = (GPS_t *)param;
    
    UNUSED(uart);

    char sentence[GPS_BUF_SIZE];

    int i = 0, j;
    while (gps->rx_buffer[i + 2] != '\0' && i + 2 < RX_BUF_SIZE) {
        memset(sentence, 0, GPS_BUF_SIZE);
        j = 0;
        while (gps->rx_buffer[i] != '\r' && j < GPS_BUF_SIZE) {
            sentence[j] = gps->rx_buffer[i];
            i++;
            j++;
        }
        sentence[j] = gps->rx_buffer[i];
        i += 2;

        #if (GPS_DEBUG == 1)
        CDC_Transmit_FS((uint8_t *)"Coucou\n", 8);
	    // CDC_Transmit_FS((uint8_t *)sentence, (uint16_t) strlen(sentence));
        HAL_Delay(10);
		#endif
        if(GPS_validate(sentence)) {
            GPS_parse(gps, sentence);
        }
    }

	// if (rx_data != '\n' && GPS->rx_index < sizeof(GPS->rx_buffer)) {
	// 	GPS->rx_buffer[GPS->rx_index++] = rx_data;
	// } else {

	// 	#if (GPS_DEBUG == 1)
	// 	GPS_print((char*)(GPS->rx_buffer));
	// 	#endif

	// 	if(GPS_validate((char*)(GPS->rx_buffer)))
	// 		GPS_parse(GPS, (char*)(GPS->rx_buffer));
	// 	GPS->rx_index = 0;
	// 	memset(GPS->rx_buffer, 0, sizeof(GPS->rx_buffer));
	// }
}

// void GPS_UART_Receive(GPS_t *GPS) {
//     uint8_t rx_data;

//     do {
// 	    HAL_UART_Receive_IT(GPS->uart, &rx_data, GPSBUFSIZE);
// 		GPS->rx_buffer[GPS->rx_index++] = rx_data;
//     } while (rx_data != '\n' && GPS->rx_index < GPSBUFSIZE);

//     #if (GPS_DEBUG == 1)
//     GPS_print((char*)(GPS->rx_buffer));
//     #endif

//     if(GPS_validate((char*)(GPS->rx_buffer)))
//         GPS_parse(GPS, (char*)(GPS->rx_buffer));
//     GPS->rx_index = 0;
//     memset(GPS->rx_buffer, 0, GPSBUFSIZE);
// }


int GPS_validate(char *nmeastr){
    char check[3];
    char checkcalcstr[3];
    int i;
    int calculated_check;

    i=0;
    calculated_check=0;

    // check to ensure that the string starts with a $
    if(nmeastr[i] == '$')
        i++;
    else
        return 0;

    //No NULL reached, 75 char largest possible NMEA message, no '*' reached
    while((nmeastr[i] != 0) && (nmeastr[i] != '*') && (i < 75)){
        calculated_check ^= nmeastr[i];// calculate the checksum
        i++;
    }

    if(i >= 75){
        return 0;// the string was too long so return an error
    }

    if (nmeastr[i] == '*'){
        check[0] = nmeastr[i+1];    //put hex chars in check string
        check[1] = nmeastr[i+2];
        check[2] = 0;
    }
    else
        return 0;// no checksum separator found there for invalid

    sprintf(checkcalcstr,"%02X",calculated_check);
    return((checkcalcstr[0] == check[0])
        && (checkcalcstr[1] == check[1])) ? 1 : 0 ;
}

void GPS_parse(GPS_t *GPS, char *GPSstrParse) {
    int nbr_elems = count_nbr_elems(GPSstrParse, ',');
    char **elems = (char **)malloc(nbr_elems * sizeof(char *));

    set_elems_from_csv(elems, GPSstrParse, ',', nbr_elems);

    // DEBUG: test elems
    // char word[255];
    // for (int i = 0; i < nbr_elems; ++i) {
    //     strcpy(word, elems[i]);
    // }
    char NMEA_type[4] = {elems[0][3], elems[0][4], elems[0][5], '\0'};
    
           if (!strcmp(NMEA_type, "GGA")) {
        GPS_parse_GGA(GPS, elems);
    } else if (!strcmp(NMEA_type, "GLL")) {
        GPS_parse_GLL(GPS, elems);
    } else if (!strcmp(NMEA_type, "VTG")) {
        GPS_parse_VTG(GPS, elems);
    }

    free(elems);
}

void GPS_parse_GLL(GPS_t *GPS, char **elems) {
    GPS->nmea_latitude  = atof(elems[1]);
    GPS->ns             =      elems[2][0];
    GPS->nmea_longitude = atof(elems[3]);
    GPS->ew             =      elems[4][0];
    GPS->utc_time       = atof(elems[5]);
    GPS->gll_status     =      elems[6][0];

    if (GPS->gll_status == 'A') {
        GPS->dec_latitude  = GPS_nmea_to_dec(GPS->nmea_latitude , GPS->ns) / 10;
        GPS->dec_longitude = GPS_nmea_to_dec(GPS->nmea_longitude, GPS->ew);
    }

    GPS->has_fix_once = GPS->has_fix_once | (!GPS->has_fix_once && GPS->gll_status == 'A');
}

void GPS_parse_GGA(GPS_t *GPS, char **elems) {
    GPS->utc_time       = atof(elems[1]);
    GPS->nmea_latitude  = atof(elems[2]);
    GPS->ns             =      elems[3][0];
    GPS->nmea_longitude = atof(elems[4]);
    GPS->ew             =      elems[5][0];
    GPS->lock           = atoi(elems[6]);
    GPS->satelites      = atoi(elems[7]);
    GPS->hdop           = atof(elems[8]);
    GPS->msl_altitude   = atof(elems[9]);
    GPS->msl_units      =      elems[10][0];

    GPS->dec_latitude  = GPS_nmea_to_dec(GPS->nmea_latitude , GPS->ns) / 10;
    GPS->dec_longitude = GPS_nmea_to_dec(GPS->nmea_longitude, GPS->ew);
}

void GPS_parse_VTG(GPS_t *GPS, char **elems) {
    GPS->course_t      = atof(elems[1]);
    GPS->course_t_unit =      elems[2][0];
    GPS->course_m      = atof(elems[3]);
    GPS->course_m_unit =      elems[4][0];
    GPS->speed_k       = atof(elems[5]);
    GPS->speed_k_unit  =      elems[6][0];
    GPS->speed_km      = atof(elems[7]);
    GPS->speed_km_unit =      elems[8][0];
}


// void GPS_parse(GPS_t *GPS, char *GPSstrParse){

//          if (!strncmp(GPSstrParse, "$GNGGA", 6)){
//     	if (sscanf(GPSstrParse, "$GNGGA,%f,%f,%c,%f,%c,%d,%d,%f,%f,%c",
//                    &(GPS->utc_time),
//                    &(GPS->nmea_latitude),
//                    &(GPS->ns),
//                    &(GPS->nmea_longitude),
//                    &(GPS->ew),
//                    &(GPS->lock),
//                    &(GPS->satelites),
//                    &(GPS->hdop),
//                    &(GPS->msl_altitude),
//                    &(GPS->msl_units)) >= 1){
//     		GPS->dec_latitude = GPS_nmea_to_dec(GPS->nmea_latitude, GPS->ns);
//     		GPS->dec_longitude = GPS_nmea_to_dec(GPS->nmea_longitude, GPS->ew);
//     		return;
//     	}
//     }
//     else if (!strncmp(GPSstrParse, "$GNRMC", 6)){
//     	if(sscanf(GPSstrParse, "$GNRMC,%f,%f,%c,%f,%c,%f,%f,%d",
//                   &(GPS->utc_time),
//                   &(GPS->nmea_latitude),
//                   &(GPS->ns),
//                   &(GPS->nmea_longitude),
//                   &(GPS->ew),
//                   &(GPS->speed_k),
//                   &(GPS->course_d),
//                   &(GPS->date)) >= 1)
//     		return;

//     }
//     else if (!strncmp(GPSstrParse, "$GNGLL", 6)){
//         if(sscanf(GPSstrParse, "$GNGLL,%f,%c,%f,%c,%f,%c",
//                   &(GPS->nmea_latitude),
//                   &(GPS->ns),
//                   &(GPS->nmea_longitude),
//                   &(GPS->ew),
//                   &(GPS->utc_time),
//                   &(GPS->gll_status)) >= 1)
//             return;
//     }
//     else if (!strncmp(GPSstrParse, "$GNVTG", 6)){
//         if(sscanf(GPSstrParse, "$GNVTG,%f,%c,%f,%c,%f,%c,%f,%c",
//                   &(GPS->course_t),
//                   &(GPS->course_t_unit),
//                   &(GPS->course_m),
//                   &(GPS->course_m_unit),
//                   &(GPS->speed_k),
//                   &(GPS->speed_k_unit),
//                   &(GPS->speed_km),
//                   &(GPS->speed_km_unit)) >= 1)
//             return;
//     }
// }

// void GPS_parse(char *GPSstrParse){
//     if(!strncmp(GPSstrParse, "$GPGGA", 6)){
//     	if (sscanf(GPSstrParse, "$GPGGA,%f,%f,%c,%f,%c,%d,%d,%f,%f,%c",
//                    &(GPS.utc_time),
//                    &(GPS.nmea_latitude),
//                    &(GPS.ns),
//                    &(GPS.nmea_longitude),
//                    &(GPS.ew),
//                    &(GPS.lock),
//                    &(GPS.satelites),
//                    &(GPS.hdop),
//                    &(GPS.msl_altitude),
//                    &(GPS.msl_units)) >= 1){
//     		GPS.dec_latitude = GPS_nmea_to_dec(GPS.nmea_latitude, GPS.ns);
//     		GPS.dec_longitude = GPS_nmea_to_dec(GPS.nmea_longitude, GPS.ew);
//     		return;
//     	}
//     }
//     else if (!strncmp(GPSstrParse, "$GPRMC", 6)){
//     	if(sscanf(GPSstrParse, "$GPRMC,%f,%f,%c,%f,%c,%f,%f,%d",
//                   &(GPS.utc_time),
//                   &(GPS.nmea_latitude),
//                   &(GPS.ns),
//                   &(GPS.nmea_longitude),
//                   &(GPS.ew),
//                   &(GPS.speed_k),
//                   &(GPS.course_d),
//                   &(GPS.date)) >= 1)
//     		return;

//     }
//     else if (!strncmp(GPSstrParse, "$GPGLL", 6)){
//         if(sscanf(GPSstrParse, "$GPGLL,%f,%c,%f,%c,%f,%c",
//                   &(GPS.nmea_latitude),
//                   &(GPS.ns),
//                   &(GPS.nmea_longitude),
//                   &(GPS.ew),
//                   &(GPS.utc_time),
//                   &(GPS.gll_status)) >= 1)
//             return;
//     }
//     else if (!strncmp(GPSstrParse, "$GPVTG", 6)){
//         if(sscanf(GPSstrParse, "$GPVTG,%f,%c,%f,%c,%f,%c,%f,%c",
//                   &(GPS.course_t),
//                   &(GPS.course_t_unit),
//                   &(GPS.course_m),
//                   &(GPS.course_m_unit),
//                   &(GPS.speed_k),
//                   &(GPS.speed_k_unit),
//                   &(GPS.speed_km),
//                   &(GPS.speed_km_unit)) >= 1)
//             return;
//     }
// }

float GPS_nmea_to_dec(float deg_coord, char nsew) {
    int degree = (int)(deg_coord/100);
    float minutes = deg_coord - degree*100;
    float dec_deg = minutes / 60;
    float decimal = degree + dec_deg;
    if (nsew == 'S' || nsew == 'W') { // return negative
        decimal *= -1;
    }
    return decimal;
}

void GPS_USBPrint(GPS_t *gps) {
    char buf[GPS_BUF_SIZE] = {0,};
    sprintf(buf, "UTC Time: %f, Latitude: %f, Longitude: %f, Altitude: %f\n",
            gps->utc_time,
            gps->dec_latitude,
            gps->dec_longitude,
            gps->msl_altitude);
    CDC_Transmit_FS((uint8_t *) buf, (uint16_t) strlen(buf));
    HAL_Delay(1);
}