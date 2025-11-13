#ifndef TRANS_UDP_HPP
#define TRANS_UDP_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <arpa/inet.h>
#include <unistd.h>

class Trans_UDP {
public:
    Trans_UDP(const std::string& server_ip, int port);
    ~Trans_UDP();

    bool initSocket();
    void sendFrame(const cv::Mat& frame, int quality = 80);
    void closeSocket();

private:
    std::string server_ip_;
    int port_;
    int sock_;
    sockaddr_in server_addr_;
};

#endif // TRANS_UDP_HPP
