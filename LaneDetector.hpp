#ifndef LANE_DETECTOR_HPP
#define LANE_DETECTOR_HPP

#include <opencv2/opencv.hpp>
#include <iostream>
#include "ComputeMpc.hpp"
#include "MpcParameters.hpp"

class LaneDetector {
private:
    ComputeMpc mpc_computer_;
    MpcParametersManager mpc_params_;
    cv::VideoCapture cap;
    int width, height;

public:
    LaneDetector(const std::string& videoPath, int width = 640, int height = 480);
    ~LaneDetector();

    void processFrame();

    cv::Mat applyIPM(cv::Mat& frame);
    void slidingWindow(const cv::Mat& mask,
                       std::vector<cv::Point>& left_points,
                       std::vector<cv::Point>& right_points,
                       cv::Mat& outImg);

    void slidingWindowAdaptive(const cv::Mat& mask,
                               std::vector<cv::Point>& lane_points,
                               cv::Mat& outImg,
                               cv::Vec3f prev_poly);

    cv::Mat processMask(const cv::Mat& bird_eye_view);
    std::vector<std::vector<cv::Point>> findContoursInMask(const cv::Mat& mask);
    cv::Vec3f fitPoly(const std::vector<cv::Point>& points, cv::Mat& outImg, bool isLeft);
    std::vector<cv::Point> computeCenterline(cv::Vec3f coeff_left,
                                            cv::Vec3f coeff_right,
                                            bool has_left, bool has_right,
                                            cv::Mat& outImg);
    float computeLaneSlope(const cv::Vec3f& coeffs, float y);
    MpcParametersManager& getMpcParameters() { return mpc_params_; }
};

#endif
