#include "TelemParser.h"
#include <iostream>

TelemParser::TelemParser(const std::string& filename) {
    csv.open(filename, std::ios::out | std::ios::app);
    if (!csv.is_open()) {
        std::cerr << "Failed to open CSV file: " << filename << std::endl;
        return;
    }

    if (csv.tellp() == 0) {
        csv << "time,press,acc_x,acc_y,acc_z,gyr_x,gyr_y,gyr_z,"
            << "mag_x,mag_y,mag_z,long_gps,lat_gps,alt_gps,volt,msg\n";
    }
}

TelemParser::~TelemParser() {
    if (csv.is_open()) {
        csv.close();
    }
}

bool TelemParser::readFromStream(std::ifstream& stream, TELEM_FORMAT_FT_APEX_DATA_MDR& data) {
    stream.read(reinterpret_cast<char*>(&data), sizeof(data));
    return stream.gcount() == sizeof(data);
}

void TelemParser::writeToCSV(const TELEM_FORMAT_FT_APEX_DATA_MDR& d) {
    if (!csv.is_open()) return;

    csv << d.time << "," << d.press << ","
        << d.acc_x << "," << d.acc_y << "," << d.acc_z << ","
        << d.gyr_x << "," << d.gyr_y << "," << d.gyr_z << ","
        << d.mag_x << "," << d.mag_y << "," << d.mag_z << ","
        << d.long_gps << "," << d.lat_gps << "," << d.alt_gps << ","
        << d.volt << "," << static_cast<int>(d.msg) << "\n";
    csv.flush();
}
