#include <iostream>
#include <fstream>
#include <string>
#include <unistd.h>

#define SERIAL_PORT "/dev/ttyACM0"  // Change to /dev/ttyACM0 if needed
#define BAUD_RATE 9600  // Match the baud rate of your Arduino

int main() {
    std::ifstream serialInput(SERIAL_PORT, std::ios::in);
    
    if (!serialInput.is_open()) {
        std::cerr << "Error: Could not open " << SERIAL_PORT << std::endl;
        return 1;
    }

    std::string message;
    while (true) {
        if (std::getline(serialInput, message)) {
            std::cout << "Received: " << message << std::endl;
        }
        usleep(100000);  // Sleep for 100ms to avoid excessive CPU usage
    }

    return 0;
}
