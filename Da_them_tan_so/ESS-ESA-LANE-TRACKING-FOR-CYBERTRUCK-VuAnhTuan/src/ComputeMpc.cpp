#include "ComputeMpc.hpp"
#include <algorithm>
#include <numeric>

const float DISTANCE_TO_AXLE = 0.15f;  // 15 cm từ đáy ảnh đến trục xe

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

    std::vector<float> x_vals, y_vals;
    for (const auto& pt : centerline) {
        x_vals.push_back(static_cast<float>(pt.x));
        y_vals.push_back(static_cast<float>(pt.y));
    }

    cv::Mat Y(y_vals.size(), 1, CV_32F, y_vals.data());
    cv::Mat X(x_vals.size(), 1, CV_32F, x_vals.data());

    cv::Mat A(Y.rows, 3, CV_32F);
    for (int i = 0; i < Y.rows; ++i) {
        float y = Y.at<float>(i, 0);
        A.at<float>(i, 0) = y * y;
        A.at<float>(i, 1) = y;
        A.at<float>(i, 2) = 1.0f;
    }

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
    return 2.0f * coeffs[0] * y + coeffs[1];
}

float ComputeMpc::computeSecondDerivative(const cv::Vec3f& coeffs) {
    return 2.0f * coeffs[0];
}

std::vector<float> ComputeMpc::computeMultipleCurvatures(const cv::Vec3f& coeffs, int N) {
    std::vector<float> curvatures;
    float a = coeffs[0];
    float b = coeffs[1];

    for (int i = 0; i < N; ++i) {
        float y = vehicle_y_ - i * 26.0f;  // 26 px ≈ 3 cm

        float dx_dy = 2.0f * a * y + b;
        float d2x_dy2 = 2.0f * a;

        float numerator = std::abs(d2x_dy2);
        float denominator = std::pow(1.0f + dx_dy * dx_dy, 1.5f);

        float kappa_pixel = (denominator > 1e-6f) ? (numerator / denominator) : 0.0f;
        float kappa_meter = kappa_pixel / pixel_per_meter_;
        curvatures.push_back(kappa_meter);
    }

    return curvatures;
}

float ComputeMpc::computeLateralDeviation(const cv::Vec3f& coeffs,
                                          const cv::Mat& birdEyeView) {
    if (vehicle_x_ == 0.0f && vehicle_y_ == 0.0f) {
        vehicle_x_ = birdEyeView.cols / 2.0f;
        vehicle_y_ = birdEyeView.rows - 1.0f;
    }

    float centerline_x = coeffs[0] * vehicle_y_ * vehicle_y_ +
                         coeffs[1] * vehicle_y_ +
                         coeffs[2];

    float lateral_deviation_pixel = vehicle_x_ - centerline_x;
    float lateral_deviation_meter = lateral_deviation_pixel * pixel_per_meter_;

    return lateral_deviation_meter;
}

float ComputeMpc::computeYawAngle(const cv::Vec3f& coeffs, float y) {
    float dx_dy = computeFirstDerivative(coeffs, y);
    return std::atan(dx_dy);
}

MpcState ComputeMpc::computeMpcParameters(const std::vector<cv::Point>& centerline,
                                          const cv::Mat& birdEyeView) {
    MpcState result;

    if (centerline.size() < 3) {
        result.is_valid = false;
        return result;
    }

    cv::Vec3f coeffs = fitCenterlinePoly(centerline);

    if (vehicle_x_ == 0.0f && vehicle_y_ == 0.0f) {
        vehicle_x_ = birdEyeView.cols / 2.0f;
        vehicle_y_ = birdEyeView.rows - 1.0f;
    }

    result.curvature = computeMultipleCurvatures(coeffs, 10);
    result.yaw_angle = computeYawAngle(coeffs, vehicle_y_);
    float raw_dev = computeLateralDeviation(coeffs, birdEyeView);
    result.lateral_deviation = raw_dev - DISTANCE_TO_AXLE * std::sin(result.yaw_angle);
    result.is_valid = true;
    return result;
}
