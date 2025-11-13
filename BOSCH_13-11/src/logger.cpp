#include "logger.hpp"
#include <iostream>

Logger::Logger(const std::string& filename) {
    file_.open(filename, std::ios::out | std::ios::trunc);
    if (!file_.is_open()) {
        std::cerr << "Cannot open log file: " << filename << std::endl;
    } else {
        file_ << "==== Performance Log ====" << std::endl;
    }
}

Logger::~Logger() {
    if (file_.is_open()) {
        file_.close();
    }
}

void Logger::log(const std::string& tag, double duration_ms) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    file_ << std::put_time(std::localtime(&time), "%H:%M:%S")
          << " [" << tag << "] "
          << std::fixed << std::setprecision(3)
          << duration_ms << " ms"
          << std::endl;
}
