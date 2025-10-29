#ifndef LANEDETECTOR_HPP
#define LANEDETECTOR_HPP

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include "ComputeMpc.hpp"
#include "MpcController.hpp"  

class LaneDetector {
private:
    cv::VideoCapture cap;
    int width;
    int height;
    ComputeMpc mpc_computer_;
    MpcState current_state_;

    float current_steering_cmd_;
    int current_servo_angle_;
    bool has_steering_info_;

public:
    LaneDetector(const std::string& videoPath, int width = 640, int height = 480);
    ~LaneDetector();

    bool getFrame(cv::Mat& frame);
    bool isOpened() const;
    
    // Xử lý frame và cập nhật current_state_
    void processFrame(cv::Mat& frame);
    
    // Lấy state hiện tại
    MpcState getMpcState() const { return current_state_; }

    // Thêm phương thức để set steering info
    void setSteeringInfo(float steering_cmd, int servo_angle) {
        current_steering_cmd_ = steering_cmd;
        current_servo_angle_ = servo_angle;
        has_steering_info_ = true;
    }
    
    cv::Mat applyIPM(cv::Mat& frame);
    cv::Mat processMask(const cv::Mat& bird_eye_view);
    
    void slidingWindow(const cv::Mat& mask,
                      std::vector<cv::Point>& left_points,
                      std::vector<cv::Point>& right_points,
                      cv::Mat& outImg);
    
    void slidingWindowAdaptive(const cv::Mat& mask,
                              std::vector<cv::Point>& lane_points,
                              cv::Mat& outImg,
                              cv::Vec3f prev_poly);
    
    std::vector<std::vector<cv::Point>> findContoursInMask(const cv::Mat& mask);
    
    cv::Vec3f fitPoly(const std::vector<cv::Point>& points, 
                     cv::Mat& outImg, 
                     bool isLeft);
    
    std::vector<cv::Point> computeCenterline(cv::Vec3f coeff_left,
                                            cv::Vec3f coeff_right,
                                            bool has_left, 
                                            bool has_right,
                                            cv::Mat& outImg);
    
    float computeLaneSlope(const cv::Vec3f& coeffs, float y);
};

#endif