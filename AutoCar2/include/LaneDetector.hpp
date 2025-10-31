#ifndef LANEDETECTOR_HPP
#define LANEDETECTOR_HPP

#include <opencv2/opencv.hpp>
#include <vector>

class LaneDetector {
public:
    LaneDetector(const std::string& videoPath, int width = 640, int height = 480);
    ~LaneDetector();

    bool getFrame(cv::Mat& frame);
    bool isOpened() const;
    void processFrame(cv::Mat& frame);
    
    // Getters for MPC computation
    std::vector<cv::Point> getCenterline() const { return centerline_; }
    cv::Mat getBirdEyeView() const { return bird_eye_view_; }
    bool hasValidLane() const { return has_valid_lane_; }
    
    // Setter for display
    void setSteeringInfo(float steering_cmd, int servo_angle) {
        current_steering_cmd_ = steering_cmd;
        current_servo_angle_ = servo_angle;
        has_steering_info_ = true;
    }
    
    // Setter for MPC display data
    void setMpcDisplayData(float curvature, float lateral_dev, float yaw_angle) {
        display_curvature_ = curvature;
        display_lateral_dev_ = lateral_dev;
        display_yaw_angle_ = yaw_angle;
        has_mpc_data_ = true;
    }

private:
    cv::VideoCapture cap;
    int width;
    int height;
    
    // Display data
    float current_steering_cmd_;
    int current_servo_angle_;
    bool has_steering_info_;
    
    float display_curvature_;
    float display_lateral_dev_;
    float display_yaw_angle_;
    bool has_mpc_data_;
    
    // Lane detection results
    std::vector<cv::Point> centerline_;
    cv::Mat bird_eye_view_;
    bool has_valid_lane_;
    
    // Lane detection methods
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
    
    cv::Vec3f fitPoly(const std::vector<cv::Point>& points, 
                     cv::Mat& outImg, 
                     bool isLeft);
    
    std::vector<cv::Point> computeCenterline(cv::Vec3f coeff_left,
                                            cv::Vec3f coeff_right,
                                            bool has_left, 
                                            bool has_right,
                                            cv::Mat& outImg);
    
    float computeLaneSlope(const cv::Vec3f& coeffs, float y);
    
    void displayInfo(cv::Mat& frame_resize, bool left_ok, bool right_ok);
};

#endif
