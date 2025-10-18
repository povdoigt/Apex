#pragma once

#include <fstream>
#include <string>
#include <cstdint>

typedef struct TELEM_FORMAT_FT_APEX_DATA_MDR {
    uint8_t sender_id;
    uint32_t time;
    float press;
    float acc_x, acc_y, acc_z;
    float gyr_x, gyr_y, gyr_z;
    float mag_x, mag_y, mag_z;
    float long_gps, lat_gps, alt_gps;
    float volt;
    uint8_t msg;
} TELEM_FORMAT_FT_APEX_DATA_MDR;

class TelemParser {
public:
    explicit TelemParser(const std::string& filename);
    ~TelemParser();

    bool readFromStream(std::ifstream& stream, TELEM_FORMAT_FT_APEX_DATA_MDR& data);
    void writeToCSV(const TELEM_FORMAT_FT_APEX_DATA_MDR& data);

private:
    std::ofstream csv;
};
