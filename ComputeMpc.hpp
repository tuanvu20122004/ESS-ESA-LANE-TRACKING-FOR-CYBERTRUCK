#ifndef COMPUTEMPC_HPP
#define COMPUTEMPC_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>

class ComputeMpc {
public:
    ComputeMpc(float pixel_per_meter = 10.0f);
    
    // Tính toán các tham số MPC từ centerline
    void computeMpcParameters(const std::vector<cv::Point>& centerline,
                             const cv::Mat& birdEyeView,
                             float& curvature,
                             float& offset,
                             float& angle_y);
    
    // Fit đường centerline với polynomial bậc 2: x = ay² + by + c
    cv::Vec3f fitCenterlinePoly(const std::vector<cv::Point>& centerline);
    
    // Tính curvature từ polynomial coefficients
    float computeCurvature(const cv::Vec3f& coeffs, float y);
    
    // Tính offset (khoảng cách từ tâm xe đến centerline)
    float computeLateralDeviation(const cv::Vec3f& coeffs, 
                                   const cv::Mat& birdEyeView);
    
    // Tính relative yaw angle (góc giữa hướng xe và tiếp tuyến centerline)
    float computeYawAngle(const cv::Vec3f& coeffs, float y);
    
    // Setter cho vehicle position (tâm xe trong bird's eye view)
    void setVehiclePosition(float x, float y);
    
private:
    float pixel_per_meter_;      // Tỉ lệ chuyển đổi pixel sang mét
    float vehicle_x_;            // Vị trí x của tâm xe (thường ở giữa ảnh)
    float vehicle_y_;            // Vị trí y của tâm xe (thường ở dưới ảnh)
    
    // Helper: Tính đạo hàm bậc 1 của x theo y
    float computeFirstDerivative(const cv::Vec3f& coeffs, float y);
    
    // Helper: Tính đạo hàm bậc 2 của x theo y
    float computeSecondDerivative(const cv::Vec3f& coeffs);
};

#endif // COMPUTEMPC_HPP