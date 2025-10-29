#ifndef COMPUTEMPC_HPP
#define COMPUTEMPC_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include "MpcController.hpp"  

class ComputeMpc {
public:
    ComputeMpc(float pixel_per_meter = 0.0002f);
    
    // Trả về MpcState: struct chứa cur, lateral, yaw
    MpcState computeMpcParameters(const std::vector<cv::Point>& centerline,
                                  const cv::Mat& birdEyeView);
    
    //fit poly bậc 2 cho centerline
    cv::Vec3f fitCenterlinePoly(const std::vector<cv::Point>& centerline);
    
    std::vector<float> computeMultipleCurvatures(const cv::Vec3f& coeffs, int N = 10);
    
    float computeLateralDeviation(const cv::Vec3f& coeffs, 
                                   const cv::Mat& birdEyeView);
    
    float computeYawAngle(const cv::Vec3f& coeffs, float y);
    
    //Gán vị trị của xe
    void setVehiclePosition(float x, float y);
    
private:
    float pixel_per_meter_;
    float vehicle_x_;
    float vehicle_y_;
    
    //đạo hàm bậc 1
    float computeFirstDerivative(const cv::Vec3f& coeffs, float y);

    //đạo hàm bậc 2
    float computeSecondDerivative(const cv::Vec3f& coeffs);
};

#endif
