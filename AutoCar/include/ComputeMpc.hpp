#ifndef COMPUTEMPC_HPP
#define COMPUTEMPC_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include "MpcState.hpp"

class ComputeMpc {
public:
    ComputeMpc(float pixel_per_meter = 0.0002f);
    
    // Trả về MpcState thay vì 3 tham số riêng
    MpcState computeMpcParameters(const std::vector<cv::Point>& centerline,
                                  const cv::Mat& birdEyeView);
    
    cv::Vec3f fitCenterlinePoly(const std::vector<cv::Point>& centerline);
    
    std::vector<float> computeMultipleCurvatures(const cv::Vec3f& coeffs, int N = 10);
    
    float computeLateralDeviation(const cv::Vec3f& coeffs, 
                                   const cv::Mat& birdEyeView);
    
    float computeYawAngle(const cv::Vec3f& coeffs, float y);
    
    void setVehiclePosition(float x, float y);
    
private:
    float pixel_per_meter_;
    float vehicle_x_;
    float vehicle_y_;
    
    float computeFirstDerivative(const cv::Vec3f& coeffs, float y);
    float computeSecondDerivative(const cv::Vec3f& coeffs);
};

#endif