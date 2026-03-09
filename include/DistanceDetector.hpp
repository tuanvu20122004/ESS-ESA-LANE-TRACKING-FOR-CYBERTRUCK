#ifndef DISTANCE_DETECTOR_HPP
#define DISTANCE_DETECTOR_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

#include <string>
#include <vector>
#include <unordered_map>
#include <deque>

class DistanceDetector {
public:
    DistanceDetector(const std::string& modelPath,
                     float focalLength,
                     int inputWidth = 640,
                     int inputHeight = 640,
                     float confThreshold = 0.2f,
                     float scoreThreshold = 0.2f,
                     float nmsThreshold = 0.2f);

    void detectAndDraw(cv::Mat& frame);

private:
    struct Detection {
        cv::Rect box;
        int class_id;
        float confidence;
    };

    cv::dnn::Net net_;

    float focal_length_;
    int input_width_;
    int input_height_;
    float conf_threshold_;
    float score_threshold_;
    float nms_threshold_;

    std::vector<std::string> class_names_;
    std::unordered_map<std::string, float> real_heights_;
    std::unordered_map<std::string, std::deque<float>> pixel_buffers_;

    cv::Mat preprocess(const cv::Mat& frame, float& scale, int& pad_w, int& pad_h);

    std::vector<Detection> postprocess(const cv::Mat& frame,
                                       const std::vector<cv::Mat>& outputs,
                                       float scale,
                                       int pad_w,
                                       int pad_h);

    float estimateDistance(float real_height, float pixel_height) const;
    float smoothPixelHeight(const std::string& class_name, float pixel_height);
};

#endif