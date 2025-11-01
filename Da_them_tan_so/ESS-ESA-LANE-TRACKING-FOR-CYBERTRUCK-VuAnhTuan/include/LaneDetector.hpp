#ifndef LANEDETECTOR_HPP
#define LANEDETECTOR_HPP

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include "ComputeMpc.hpp"
#include "MpcState.hpp"
#include <numeric>

class LaneDetector {
private:
    cv::VideoCapture cap;
    int width, height;
    ComputeMpc mpc_computer_;
    MpcState current_state_;
    cv::Mat mask;
    cv::Mat bird_eye_view;
    cv::Mat frame_resize;
public:
    LaneDetector(const std::string& videoPath, int width, int height);
    ~LaneDetector();

    bool getFrame(cv::Mat& frame);
    bool isOpened() const;
    void processFrame(cv::Mat& frame);
    //MpcState getMpcState() const { return current_state_; }
    MpcState getMpcState() const;
    cv::Mat get_mask() const;
    cv::Mat get_bird_eye_view() const;
    cv::Mat get_frame_resize();
private:
    // --- Core functions ---
    cv::Mat applyIPM(cv::Mat& frame);
    cv::Mat processMask(const cv::Mat& bird_eye_view);
    void slidingWindow(const cv::Mat& mask, std::vector<cv::Point>& left_points, std::vector<cv::Point>& right_points, cv::Mat& outImg);

    // --- Advanced functions ---
    void slidingWindowAdaptive(const cv::Mat& mask, std::vector<cv::Point>& lane_points, cv::Mat& outImg, cv::Vec3f prev_poly);
    std::vector<std::vector<cv::Point>> findContoursInMask(const cv::Mat& mask);
    cv::Vec3f fitPoly(const std::vector<cv::Point>& points, cv::Mat& outImg, bool isLeft);
    std::vector<cv::Point> computeCenterline(cv::Vec3f coeff_left, cv::Vec3f coeff_right, bool has_left, bool has_right, cv::Mat& outImg);
    float computeLaneSlope(const cv::Vec3f& coeffs, float y);
};

#endif
