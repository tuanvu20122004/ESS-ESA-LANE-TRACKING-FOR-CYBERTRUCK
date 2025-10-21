#ifndef LOGIC_HPP
#define LOGIC_HPP

#include "LaneDetector.hpp"
#include "MpcController.hpp"
#include "communication.hpp"
#include "logger.hpp"

#include <opencv2/opencv.hpp>
#include <atomic>
#include <mutex>
#include <string>

class Logic {
public:
    explicit Logic(const std::string& videoPath);
    void run();

private:
    // Core modules
    LaneDetector   detector;
    MpcController  mpc;
    Communication  comm;

    // Control params
    const float desired_velocity = 0.3f;

    // Concurrency
    std::atomic<bool> running{true};
    std::mutex        frame_mutex;
    cv::Mat           latest_frame;
};

#endif // LOGIC_HPP
