#ifndef TRANS_UDP_HPP
#define TRANS_UDP_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <arpa/inet.h>
#include <unistd.h>
#include <atomic>

class Trans_UDP {
public:
    Trans_UDP(const std::string& server_ip, int port);
    ~Trans_UDP();

    bool initSocket();
    void sendFrame(const cv::Mat& frame, int quality = 80);

    // Nhận khoảng cách từ laptop
    void receiveDistance();
    float getDistance() const;

    void closeSocket();

private:
    std::string server_ip_;
    int port_;

    // socket gửi frame
    int sock_;
    sockaddr_in server_addr_;

    // socket nhận distance
    int recv_sock_;
    sockaddr_in recv_addr_;

    std::atomic<float> distance_{100.0f};
};

#endif