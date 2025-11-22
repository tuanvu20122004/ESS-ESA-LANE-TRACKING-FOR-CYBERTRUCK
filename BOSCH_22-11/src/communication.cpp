#include "communication.hpp"
#include <iostream>
#include <sstream>
#include <iomanip>

Communication::Communication(const std::string& port, unsigned int baudrate) {
    try {
        serial_port_.Open(port);

        // Cài baudrate
        switch (baudrate) {
            case 9600:
                serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_9600);
                break;
            case 115200:
                serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
                break;
            default:
                serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        }
        serial_port_.SetSerialPortBlockingStatus(false);
        std::cout << "UART connected on " << port
                  << "at " << baudrate << " baud.\n";
    }
    catch (const LibSerial::OpenFailed& e) {
        std::cerr << "Failed to open port: " << e.what() << "\n";
    }
}

Communication::~Communication() {
    if (serial_port_.IsOpen()) {
        serial_port_.Close();
        std::cout << "UART connection closed.\n";
    }
}

void Communication::sendCommands(float speed, int angle) {
    if (!serial_port_.IsOpen()) return;

    std::lock_guard<std::mutex> lock(mutex_);
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);
    oss << "CMD," << speed << "," << angle << "\r\n";

    try {
        serial_port_.Write(oss.str());
        //serial_port_.FlushOutputBuffer();
        //std::cout << "[TX] " << oss.str();
    } catch (const std::exception& e) {
        std::cerr << "UART write error: " << e.what() << "\n";
    }
}
