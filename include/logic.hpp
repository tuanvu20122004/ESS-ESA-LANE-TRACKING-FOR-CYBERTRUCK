#ifndef LOGIC_HPP
#define LOGIC_HPP

#include "LaneDetector.hpp"
#include "MpcController.hpp"
#include "communication.hpp"
#include "logger.hpp"

class Logic {
public:
    Logic(const std::string& videoPath = "Demo.mp4");
    void run();

private:
    LaneDetector detector;
    MpcController mpc;
    Communication comm;
    const float desired_velocity = 0.3f;
};

#endif
