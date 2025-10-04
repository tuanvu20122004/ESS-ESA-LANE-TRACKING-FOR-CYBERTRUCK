#include "ComputeMpc.hpp"
#include <algorithm>
#include <numeric>

ComputeMpc::ComputeMpc(float pixel_per_meter) 
    : pixel_per_meter_(pixel_per_meter),
      vehicle_x_(0.0f),
      vehicle_y_(0.0f) {
}

void ComputeMpc::setVehiclePosition(float x, float y) {
    vehicle_x_ = x;
    vehicle_y_ = y;
}

cv::Vec3f ComputeMpc::fitCenterlinePoly(const std::vector<cv::Point>& centerline) {
    if (centerline.size() < 3) {
        return cv::Vec3f(0, 0, 0);
    }
    
    // Chuẩn bị dữ liệu cho fitting
    std::vector<float> x_vals, y_vals;
    for (const auto& pt : centerline) {
        x_vals.push_back(static_cast<float>(pt.x));
        y_vals.push_back(static_cast<float>(pt.y));
    }
    
    cv::Mat Y(y_vals.size(), 1, CV_32F, y_vals.data());
    cv::Mat X(x_vals.size(), 1, CV_32F, x_vals.data());
    
    // Tạo ma trận A cho polynomial bậc 2: x = ay² + by + c
    cv::Mat A(Y.rows, 3, CV_32F);
    for (int i = 0; i < Y.rows; ++i) {
        float y = Y.at<float>(i, 0);
        A.at<float>(i, 0) = y * y;
        A.at<float>(i, 1) = y;
        A.at<float>(i, 2) = 1.0f;
    }
    
    // Giải hệ phương trình bằng SVD
    cv::Mat coeffs;
    bool ok = cv::solve(A, X, coeffs, cv::DECOMP_SVD);
    
    if (ok) {
        return cv::Vec3f(coeffs.at<float>(0), 
                        coeffs.at<float>(1), 
                        coeffs.at<float>(2));
    }
    
    return cv::Vec3f(0, 0, 0);
}

float ComputeMpc::computeFirstDerivative(const cv::Vec3f& coeffs, float y) {
    // dx/dy = 2ay + b
    return 2.0f * coeffs[0] * y + coeffs[1];
}

float ComputeMpc::computeSecondDerivative(const cv::Vec3f& coeffs) {
    // d²x/dy² = 2a
    return 2.0f * coeffs[0];
}

float ComputeMpc::computeCurvature(const cv::Vec3f& coeffs, float y) {
    // Công thức curvature cho đường cong x = f(y):
    // κ = |d²x/dy²| / (1 + (dx/dy)²)^(3/2)
    
    float dx_dy = computeFirstDerivative(coeffs, y);
    float d2x_dy2 = computeSecondDerivative(coeffs);
    
    float numerator = std::abs(d2x_dy2);
    float denominator = std::pow(1.0f + dx_dy * dx_dy, 1.5f);
    
    if (denominator < 1e-6f) {
        return 0.0f;
    }
    
    // Chuyển đổi từ pixel sang mét
    float curvature_pixel = numerator / denominator;
    float curvature_meter = curvature_pixel * pixel_per_meter_;
    
    return curvature_meter;
}

float ComputeMpc::computeLateralDeviation(const cv::Vec3f& coeffs, 
                                          const cv::Mat& birdEyeView) {
    // Mặc định vị trí xe ở giữa dưới cùng của bird's eye view
    if (vehicle_x_ == 0.0f && vehicle_y_ == 0.0f) {
        vehicle_x_ = birdEyeView.cols / 2.0f;
        vehicle_y_ = birdEyeView.rows - 1.0f;
    }
    
    // Tính x của centerline tại vị trí y của xe
    float centerline_x = coeffs[0] * vehicle_y_ * vehicle_y_ + 
                         coeffs[1] * vehicle_y_ + 
                         coeffs[2];
    
    // Lateral deviation = khoảng cách từ xe đến centerline
    // Âm: xe lệch trái, Dương: xe lệch phải
    float lateral_deviation_pixel = vehicle_x_ - centerline_x;
    
    // Chuyển đổi sang mét
    float lateral_deviation_meter = lateral_deviation_pixel / pixel_per_meter_;
    
    return lateral_deviation_meter;
}

float ComputeMpc::computeYawAngle(const cv::Vec3f& coeffs, float y) {
    // Góc yaw tương đối = arctan(dx/dy)
    // Đây là góc giữa trục dọc (y) và tiếp tuyến của centerline
    
    float dx_dy = computeFirstDerivative(coeffs, y);
    
    // atan2 trả về góc trong khoảng [-π, π]
    float yaw_angle_rad = std::atan(dx_dy);
    
    // Chuyển sang độ (nếu cần)
    // float yaw_angle_deg = yaw_angle_rad * 180.0f / M_PI;
    
    return yaw_angle_rad; // Trả về radian theo chuẩn MPC
}

void ComputeMpc::computeMpcParameters(const std::vector<cv::Point>& centerline,
                                     const cv::Mat& birdEyeView,
                                     float& curvature,
                                     float& offset,
                                     float& angle_y) {
    // Kiểm tra centerline có đủ điểm không
    if (centerline.size() < 3) {
        curvature = 0.0f;
        offset = 0.0f;
        angle_y = 0.0f;
        return;
    }
    
    // 1. Fit polynomial cho centerline
    cv::Vec3f coeffs = fitCenterlinePoly(centerline);
    
    // 2. Xác định điểm đánh giá (thường là vị trí xe)
    if (vehicle_x_ == 0.0f && vehicle_y_ == 0.0f) {
        vehicle_x_ = birdEyeView.cols / 2.0f;
        vehicle_y_ = birdEyeView.rows - 1.0f;
    }
    
    // 3. Tính curvature tại vị trí xe
    curvature = computeCurvature(coeffs, vehicle_y_);
    
    // 4. Tính lateral deviation
    offset = computeLateralDeviation(coeffs, birdEyeView);
    
    // 5. Tính relative yaw angle
    angle_y = computeYawAngle(coeffs, vehicle_y_);
}