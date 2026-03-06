#include "Trans_UDP.hpp"
#include <iostream>
#include <vector>
#include <arpa/inet.h>
#include <unistd.h>

Trans_UDP::Trans_UDP(const std::string& server_ip, int port)
    : server_ip_(server_ip), port_(port), sock_(-1)
{
    initSocket();
}

Trans_UDP::~Trans_UDP() {
    closeSocket();
}

bool Trans_UDP::initSocket() {
    sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_ < 0) {
        std::cerr << "Không tạo được socket UDP\n";
        return false;
    }

    server_addr_.sin_family = AF_INET;
    server_addr_.sin_port = htons(port_);
    inet_pton(AF_INET, server_ip_.c_str(), &server_addr_.sin_addr);

    std::cout << "Socket UDP đã khởi tạo (→ " 
              << server_ip_ << ":" << port_ << ")\n";
    return true;
}

void Trans_UDP::sendFrame(const cv::Mat& frame, int quality) {
    if (sock_ < 0 || frame.empty()) return;

    std::vector<uchar> buf;
    std::vector<int> params = { cv::IMWRITE_JPEG_QUALITY, quality };
    cv::imencode(".jpg", frame, buf, params);

    sendto(sock_, buf.data(), buf.size(), 0,
           reinterpret_cast<sockaddr*>(&server_addr_), sizeof(server_addr_));
}

void Trans_UDP::closeSocket() {
    if (sock_ > 0) {
        close(sock_);
    }
    sock_ = -1;
}
