#ifndef DATA_PUBLISHER_H
#define DATA_PUBLISHER_H

#include "stm32f4xx_hal.h"
#include "utils/circular_buffer.h"

typedef struct FLOAT3 {
    float x;
    float y;
    float z;
} FLOAT3;

typedef struct FLOAT1_TIMESTAMP {
    float data;
    uint32_t timestamp; // in ms
} FLOAT1_TIMESTAMP;

typedef struct FLOAT3_TIMESTAMP {
    FLOAT3 data;
    uint32_t timestamp; // in ms
} FLOAT3_TIMESTAMP;

typedef struct DATA_PUB_WRITTER CIRCULAR_BUFFER_WRITTER DATA_PUB_WRITTER;

typedef struct DATA_PUB_READ {
    DATA_PUB_WRITTER *dpw;
    size_t new_data_num; // Number of new data available to read
} DATA_PUB_READ;

typedef struct DATA_ALL_TIMESTAMP {
    // FLOAT3_TIMESTAMP adxl375_acc;        // ADXL375 Acc

    uint16_t dummy_start;

    FLOAT3_TIMESTAMP bmi088_acc;    // BMI088 Acc
    FLOAT3_TIMESTAMP bmi088_gyr;    // BMI088 Gyr

    // FLOAT1_TIMESTAMP bmp380_temp;   // Temperature in Celsius
    // FLOAT1_TIMESTAMP bmp380_pres;   // Pressure in hPa

    // FLOAT3_TIMESTAMP lsm303agr_acc;        // LSM303AGR Acc
    // FLOAT3_TIMESTAMP lsm303agr_mag;        // LSM303AGR Mag

    float longitude;    // Longitude in degrees
    float latitude;     // Latitude in degrees
    float altitude;     // Altitude in meters
    float gps_time;     // GPS time in hhmmss.ss

    uint32_t timestamp; // Timestamp in ms
    uint16_t dummy_end;
}__attribute__((packed)) DATA_ALL_TIMESTAMP;

typedef struct DATA_ALL_TIMESTAMP_2m {
    // FLOAT3_TIMESTAMP adxl375_acc;        // ADXL375 Acc

    uint16_t dummy_start;

    FLOAT3 bmi088_acc;    // BMI088 Acc
    FLOAT3 bmi088_gyr;    // BMI088 Gyr

    // FLOAT1_TIMESTAMP bmp380_temp;   // Temperature in Celsius
    // FLOAT1_TIMESTAMP bmp380_pres;   // Pressure in hPa

    // FLOAT3_TIMESTAMP lsm303agr_acc;        // LSM303AGR Acc
    // FLOAT3_TIMESTAMP lsm303agr_mag;        // LSM303AGR Mag

    float longitude;    // Longitude in degrees
    float latitude;     // Latitude in degrees
    float altitude;     // Altitude in meters
    float gps_time;     // GPS time in hhmmss.ss

    uint32_t timestamp; // Timestamp in ms
    uint16_t dummy_end;
}__attribute__((packed)) DATA_ALL_TIMESTAMP_2;


typedef struct DATA_ALL_PUB {
    DATA_PUB adxl375_acc_data_pub;  // Acceleration data from ADXL375

    DATA_PUB bmi088_acc_data_pub;  // Acceleration data from BMI088
    DATA_PUB bmi088_gyr_data_pub;  // Gyroscope data from BMI088

    DATA_PUB bmp380_temp_data_pub;  // Temperature data from BMP380
    DATA_PUB bmp380_pres_data_pub;  // Pressure data from BMP380

    DATA_PUB lsm303agr_acc_data_pub;  // Acceleration data from LSM303AGR
    DATA_PUB lsm303agr_mag_data_pub;  // Magnetometer data from LSM303AGR

    // DATA_PUB gps_longitude_data_pub;  // Longitude data from GPS
    // DATA_PUB gps_latitude_data_pub;   // Latitude data from GPS
    // DATA_PUB gps_altitude_data_pub;   // Altitude data from GPS
} DATA_ALL_PUB;

#define DATA_PUB_SIZE(data_type, capacity) (sizeof(DATA_PUB) + sizeof(data_type) * capacity)

void DATA_PUB_init(DATA_PUB_WRITTER *dpw, uint8_t *buffer, size_t data_size, size_t capacity);
void DATA_PUB_push(DATA_PUB_READ *dpr, void *data);
void DATA_PUB_pop(DATA_PUB_READ *dpr, void *data);

#endif // DATA_PUBLISHER_H