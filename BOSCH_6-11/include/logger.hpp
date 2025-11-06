#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <fstream>
#include <mutex>
#include <string>
#include <chrono>
#include <iomanip>

class Logger {
public:
    explicit Logger(const std::string& filename);
    ~Logger();
    void log(const std::string& tag, double duration_ms);

private:
    std::ofstream file_;
    std::mutex mutex_;
};

#endif
