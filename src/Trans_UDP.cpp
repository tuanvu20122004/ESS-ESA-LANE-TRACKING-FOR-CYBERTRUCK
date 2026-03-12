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

    if (sock_ < 0)
    {
        std::cerr << "Khong tao duoc socket UDP sender\n";
        return false;
    }

    std::memset(&server_addr_, 0, sizeof(server_addr_));
    server_addr_.sin_family = AF_INET;
    server_addr_.sin_port = htons(port_);

    if (inet_pton(AF_INET, server_ip_.c_str(), &server_addr_.sin_addr) <= 0)
    {
        std::cerr << "IP server khong hop le: " << server_ip_ << std::endl;
        return false;
    }

    std::cout << "UDP sender ready -> "
              << server_ip_ << ":" << port_ << std::endl;

    return true;
}

void Trans_UDP::sendFrame(const cv::Mat& frame, int quality)
{
    if (sock_ < 0 || frame.empty()) return;

    std::vector<uchar> buf;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, quality};

    static int saved = 0;
    if (saved == 0) {
    cv::imwrite("pi_before_send.jpg", frame);
    std::cout << "Saved pi_before_send.jpg" << std::endl;
    saved = 1;
    }

    if (!cv::imencode(".jpg", frame, buf, params))
        return;

    sendto(sock_,
           buf.data(),
           buf.size(),
           0,
           reinterpret_cast<sockaddr*>(&server_addr_),
           sizeof(server_addr_));
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