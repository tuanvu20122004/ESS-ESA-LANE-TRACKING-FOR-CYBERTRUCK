#ifndef COMMUNICATION_HPP
#define COMMUNICATION_HPP

#include <libserial/SerialPort.h>
#include <string>
#include <mutex>

class Communication {
public:
    Communication(const std::string& port = "/dev/ttyACM0", unsigned int baudrate = 115200);
    ~Communication();

    void sendCommands(float speed, int angle);

private:
    LibSerial::SerialPort serial_port_;
    std::mutex mutex_;
};

#endif
