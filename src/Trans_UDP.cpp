#include "Trans_UDP.hpp"
#include <iostream>
#include <vector>
#include <cstring>
#include <cstdlib>
#include <sys/socket.h>

Trans_UDP::Trans_UDP(const std::string& server_ip, int port)
    : server_ip_(server_ip), port_(port), sock_(-1), recv_sock_(-1)
{
    initSocket();

    // ===== Tạo socket nhận khoảng cách =====
    recv_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (recv_sock_ < 0)
    {
        std::cerr << "Create UDP receiver socket failed\n";
        return;
    }

    std::memset(&recv_addr_, 0, sizeof(recv_addr_));
    recv_addr_.sin_family = AF_INET;
    recv_addr_.sin_addr.s_addr = INADDR_ANY;
    recv_addr_.sin_port = htons(8888);   // Pi nhận distance ở port 8888

    int opt = 1;
    setsockopt(recv_sock_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    if (bind(recv_sock_, reinterpret_cast<sockaddr*>(&recv_addr_), sizeof(recv_addr_)) < 0)
    {
        std::cerr << "Bind UDP receiver failed at port 8888\n";
    }
    else
    {
        std::cout << "UDP distance receiver ready at port 8888\n";
    }
}

Trans_UDP::~Trans_UDP()
{
    closeSocket();
}

bool Trans_UDP::initSocket()
{
    sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_ < 0) {
        std::cerr << "Không tạo được socket UDP\n";
        return false;
    }

    server_addr_.sin_family = AF_INET;
    server_addr_.sin_port = htons(port_);

    if (inet_pton(AF_INET, server_ip_.c_str(), &server_addr_.sin_addr) <= 0)
    {
        std::cerr << "IP server khong hop le: " << server_ip_ << std::endl;
        return false;
    }

    std::cout << "Socket UDP đã khởi tạo (→ " 
              << server_ip_ << ":" << port_ << ")\n";
    return true;
}


void Trans_UDP::sendFrame(const cv::Mat& frame, int quality)
{
    if (sock_ < 0 || frame.empty()) return;

    std::vector<uchar> buf;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, quality};

    if (!cv::imencode(".jpg", frame, buf, params))
        return;

    sendto(sock_,
           buf.data(),
           buf.size(),
           0,
           reinterpret_cast<sockaddr*>(&server_addr_),
           sizeof(server_addr_));
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


void Trans_UDP::receiveDistance()
{
    if (recv_sock_ < 0) return;

    char buffer[64] = {0};

    int n = recvfrom(recv_sock_,
                     buffer,
                     sizeof(buffer) - 1,
                     MSG_DONTWAIT,
                     NULL,
                     NULL);

    if (n > 0)
    {
        buffer[n] = '\0';

        float d = std::atof(buffer);
        distance_.store(d);
    }
}

float Trans_UDP::getDistance() const
{
    return distance_.load();
}

void Trans_UDP::closeSocket()
{
    if (sock_ >= 0)
        close(sock_);

    if (recv_sock_ >= 0)
        close(recv_sock_);

    sock_ = -1;
    recv_sock_ = -1;
}
