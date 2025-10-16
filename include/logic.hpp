#ifndef LOGIC_HPP
#define LOGIC_HPP

#include "LaneDetector.hpp"
#include "MpcController.hpp"
#include "communication.hpp"
#include "logger.hpp"
#include <opencv2/opencv.hpp>

class Logic {
public:
    Logic(const std::string& videoPath);
    void run();

private:
    bool initCamera(const std::string& source);

    LaneDetector detector;
    MpcController mpc;
    Communication comm;
    cv::VideoCapture cap;
    const float desired_velocity = 0.3f;
};

#endif
