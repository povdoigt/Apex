#ifndef UTILS_TELEM_FORMAT_H
#define UTILS_TELEM_FORMAT_H

#include "stm32f4xx_hal.h"

#include "utils/data_topic.h"
#include "utils/types.h"

typedef struct TELEM_FORMAT_MSG {
    uint8_t sender_id;      // ID of the sender
    uint8_t receiver_id;    // ID of the receiver
    uint8_t format_code;    // Telemetry format code
    uint8_t data[253];      // Data payload (max 253 bytes)
} TELEM_FORMAT_MSG;


// Telemetry format modules code
typedef enum TELEM_FORMAT_MC {
    TELEM_FORMAT_MC_GROUND_STATION  = 0x00, // Ground Station telemetry format
    TELEM_FORMAT_MC_APEX            = 0x01, // APEX telemetry format
    TELEM_FORMAT_MC_ALBA            = 0x02, // ALBA telemetry format
} TELEM_FORMAT_MC;


// === APEX Telemetry Format ===

// Telemetry format codes for APEX
typedef enum TELEM_FORMAT_TF_APEX {
    TELEM_FORMAT_TF_APEX_DATA_RAW   = 0x00, // APEX telemetry data raw format
    TELEM_FORMAT_TF_APEX_LOG        = 0x01, // APEX telemetry log format
} TELEM_FORMAT_TF_APEX;

// APEX telemetry data raw format structure
typedef struct TELEM_FORMAT_FT_APEX_DATA_RAW {
    uint32_t time;

    float temperature;  // Temperature in Celsius
    float pressure;     // Pressure in hPa

    float3_t acc0;        // Acc 0
    float3_t gyr0;        // Gyr 0

    float3_t acc1;        // Acc 1

    float3_t acc2;        // Acc 2
    float3_t mag2;        // Mag 2

    // float longitude;    // Longitude in degrees
    // float latitude;     // Latitude in degrees
    // float altitude;     // Altitude in meters
} TELEM_FORMAT_FT_APEX_DATA_RAW;


// === ALBA Telemetry Format ===

// Telemetry format codes for ALBA
typedef enum TELEM_FORMAT_TF_ALBA {
    TELEM_FORMAT_TF_ALBA_DATA_RAW   = 0x00, // ALBA telemetry data raw format
    TELEM_FORMAT_TF_ALBA_LOG        = 0x01, // ALBA telemetry log format
} TELEM_FORMAT_ALBA;

// ALBA telemetry data raw format structure
typedef struct TELEM_FORMAT_FT_ALBA_DATA_RAW {
    uint8_t delamerde; // Dummy field becaude je ne sais pas ce que tu veux mettre ici
    // ...
} TELEM_FORMAT_FT_ALBA_DATA_RAW;
// (les strcutures des formats de code ne doivent pas dépasser 253 octets)

#endif // UTILS_TELEM_FORMAT_H
