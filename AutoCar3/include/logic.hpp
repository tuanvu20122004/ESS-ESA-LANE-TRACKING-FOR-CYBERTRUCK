#ifndef LOGIC_HPP
#define LOGIC_HPP

#include "LaneDetector.hpp"
#include "MpcController.hpp"
#include "communication.hpp"
#include "logger.hpp"
#include "Trans_UDP.hpp"

#include <opencv2/opencv.hpp>
#include <atomic>
#include <mutex>
#include <string>

class Logic {
public:
    explicit Logic(const std::string& videoPath);
    void run();

private:
    //CONSTRUCTOR
    LaneDetector   detector; // v
    MpcController  mpc; //?
    Communication  comm;  //v
    Trans_UDP      udp_send; //v
    Trans_UDP      udp_send1; // v
    Logger         logger; //v
    //VELOCITY 
    const float desired_velocity = 0.05f;

    //SAFETY AND FRAME
    std::atomic<bool> running{true};
    std::mutex        frame_mutex;
    cv::Mat           latest_frame;
};

#endif // LOGIC_HPP