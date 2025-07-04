#include <iostream>
#include <fstream>
#include <string>
#include <unistd.h>

#define SERIAL_PORT "/dev/ttyACM0"  // Change to /dev/ttyACM0 if needed
#define BAUD_RATE 9600  // Match the baud rate of your Arduino

typedef struct TELEM_FORMAT_FT_APEX_DATA_MDR {
    uint8_t sender_id;      // ID of the sender

    uint32_t time;

    float press;

    float acc_x;
    float acc_y;
    float acc_z;

    float gyr_x;
    float gyr_y;
    float gyr_z;

    float mag_x;
    float mag_y;
    float mag_z;

    float long_gps;
    float lat_gps;
    float alt_gps;

    float volt;

    uint8_t msg;
} TELEM_FORMAT_FT_APEX_DATA_MDR;

int main() {
    std::ifstream serialInput(SERIAL_PORT, std::ios::in);
    
    if (!serialInput.is_open()) {
        std::cerr << "Error: Could not open " << SERIAL_PORT << std::endl;
        return 1;
    }

    std::TELEM_FORMAT_FT_APEX_DATA_MDR message;
    while (true) {
        if (std::getline(serialInput, message)) {
            std::cout << "Received: " << message << std::endl;
        }
        usleep(100000);  // Sleep for 100ms to avoid excessive CPU usage
    }

    return 0;
}
