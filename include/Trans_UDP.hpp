#ifndef TRANS_UDP_HPP
#define TRANS_UDP_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <arpa/inet.h>
#include <unistd.h>

class Trans_UDP {
public:
    Trans_UDP(const std::string& server_ip, int port, size_t buffer_size = 10);
    ~Trans_UDP();

    bool initSocket();
    void sendFrame(const cv::Mat& frame, int quality = 80);
    void closeSocket();
    bool receiveDistance();


private:

    std::string server_ip_;
    int port_;
    int sock_;
    sockaddr_in server_addr_;
    sockaddr_in local_addr_;
    std::deque<float> distance_buffer_;
    size_t buffer_size_;
    
};

#endif // TRANS_UDP_HPP