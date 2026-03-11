#include "Trans_UDP.hpp"
#include <iostream>
#include <vector>
#include <arpa/inet.h>
#include <unistd.h>

Trans_UDP::Trans_UDP(const std::string& server_ip, int port, size_t buffer_size)
    : server_ip_(server_ip), port_(port), sock_(-1), buffer_size_(buffer_size)
{
    initSocket();
}

Trans_UDP::~Trans_UDP() {
    closeSocket();
}

bool Trans_UDP::initSocket() {
    sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_ < 0) {
        std::cerr << "Khong tao duoc socket UDP\n";
        return false;
    }

    // Cau hinh dia chi server de gui frame
    std::memset(&server_addr_, 0, sizeof(server_addr_));
    server_addr_.sin_family = AF_INET;
    server_addr_.sin_port = htons(port_);
    inet_pton(AF_INET, server_ip_.c_str(), &server_addr_.sin_addr);

    // Bind local port de co the nhan distance
    std::memset(&local_addr_, 0, sizeof(local_addr_));
    local_addr_.sin_family = AF_INET;
    local_addr_.sin_port = htons(port_);
    local_addr_.sin_addr.s_addr = INADDR_ANY;

    if (bind(sock_, reinterpret_cast<sockaddr*>(&local_addr_), sizeof(local_addr_)) < 0) {
        std::cerr << "Bind that bai tren cong " << port_ << "\n";
        close(sock_);
        sock_ = -1;
        return false;
    }

    std::cout << "UDP socket da khoi tao (send -> "
              << server_ip_ << ":" << port_
              << ", recv <- 0.0.0.0:" << port_ << ")\n";

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

bool Trans_UDP::receiveDistance() {
    if (sock_ < 0) return false;

    float distance = -1.0f;
    sockaddr_in sender_addr{};
    socklen_t sender_len = sizeof(sender_addr);

    ssize_t len = recvfrom(sock_,
                           &distance,
                           sizeof(distance),
                           MSG_DONTWAIT,
                           reinterpret_cast<sockaddr*>(&sender_addr),
                           &sender_len);

    if (len < 0) {
        return false; // khong co data
    }

    if (len != sizeof(float)) {
        std::cerr << "Nhan sai kich thuoc goi tin distance: " << len << " bytes\n";
        return false;
    }

    distance_buffer_.push_back(distance);

    if (distance_buffer_.size() > buffer_size_) {
        distance_buffer_.pop_front();
    }

    return true;
}

float Trans_UDP::getLatestDistance() const {
    if (distance_buffer_.empty()) return -1.0f;
    return distance_buffer_.back();
}

float Trans_UDP::getAverageDistance() const {
    if (distance_buffer_.empty()) return -1.0f;

    float sum = 0.0f;
    for (float d : distance_buffer_) {
        sum += d;
    }
    return sum / static_cast<float>(distance_buffer_.size());
}

const std::deque<float>& Trans_UDP::getBuffer() const {
    return distance_buffer_;
}


void Trans_UDP::closeSocket() {
    if (sock_ > 0) {
        close(sock_);
    }
    sock_ = -1;
}
