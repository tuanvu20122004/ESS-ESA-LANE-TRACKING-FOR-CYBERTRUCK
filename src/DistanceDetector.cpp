#include "DistanceDetector.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

DistanceDetector::DistanceDetector(const std::string& modelPath,
                                   float focalLength,
                                   int inputWidth,
                                   int inputHeight,
                                   float confThreshold,
                                   float scoreThreshold,
                                   float nmsThreshold)
    : focal_length_(focalLength),
      input_width_(inputWidth),
      input_height_(inputHeight),
      conf_threshold_(confThreshold),
      score_threshold_(scoreThreshold),
      nms_threshold_(nmsThreshold)
{
    net_ = cv::dnn::readNet(modelPath);

    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

    class_names_ = {
        "person","bicycle","car","motorcycle","airplane","bus","train","truck","boat",
        "traffic light","fire hydrant","stop sign","parking meter","bench","bird","cat","dog",
        "horse","sheep","cow","elephant","bear","zebra","giraffe","backpack","umbrella",
        "handbag","tie","suitcase","frisbee","skis","snowboard","sports ball","kite",
        "baseball bat","baseball glove","skateboard","surfboard","tennis racket","bottle",
        "wine glass","cup","fork","knife","spoon","bowl","banana","apple","sandwich",
        "orange","broccoli","carrot","hot dog","pizza","donut","cake","chair","couch",
        "potted plant","bed","dining table","toilet","tv","laptop","mouse","remote",
        "keyboard","cell phone","microwave","oven","toaster","sink","refrigerator","book",
        "clock","vase","scissors","teddy bear","hair drier","toothbrush"
    };

    // Chỉ giữ các class muốn đo khoảng cách
    real_heights_ = {
        {"person", 1.70f},
        {"bottle", 0.25f},
        {"car", 0.185f},
        {"cell phone", 0.14f},
        {"book", 0.24f},
        {"laptop", 0.22f},
        {"chair", 0.90f}
    };

    std::cout << "[DISTANCE] Model loaded: " << modelPath << std::endl;
}

cv::Mat DistanceDetector::preprocess(const cv::Mat& frame, float& scale, int& pad_w, int& pad_h)
{
    int w = frame.cols;
    int h = frame.rows;

    scale = std::min(static_cast<float>(input_width_) / static_cast<float>(w),
                     static_cast<float>(input_height_) / static_cast<float>(h));

    int new_w = static_cast<int>(std::round(w * scale));
    int new_h = static_cast<int>(std::round(h * scale));

    cv::Mat resized;
    cv::resize(frame, resized, cv::Size(new_w, new_h));

    cv::Mat canvas(input_height_, input_width_, CV_8UC3, cv::Scalar(114, 114, 114));

    pad_w = (input_width_ - new_w) / 2;
    pad_h = (input_height_ - new_h) / 2;

    resized.copyTo(canvas(cv::Rect(pad_w, pad_h, new_w, new_h)));

    cv::Mat blob;
    cv::dnn::blobFromImage(canvas, blob, 1.0 / 255.0,
                           cv::Size(input_width_, input_height_),
                           cv::Scalar(), true, false);

    return blob;
}

std::vector<DistanceDetector::Detection>
DistanceDetector::postprocess(const cv::Mat& frame,
                              const std::vector<cv::Mat>& outputs,
                              float scale,
                              int pad_w,
                              int pad_h)
{
    std::vector<Detection> detections;
    if (outputs.empty()) return detections;

    cv::Mat out = outputs[0];

    // YOLOv8 ONNX thường ra [1,84,8400]
    if (out.dims == 3) {
        const int rows = out.size[1];
        const int cols = out.size[2];

        if (rows == 84) {
            cv::Mat reshaped(84, cols, CV_32F, out.ptr<float>());
            out = reshaped.t();  // -> [8400,84]
        } else if (cols == 84) {
            out = cv::Mat(rows, 84, CV_32F, out.ptr<float>()).clone();
        }
    }

    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;
    std::vector<int> class_ids;

    for (int i = 0; i < out.rows; ++i) {
        const float* data = out.ptr<float>(i);

        float cx = data[0];
        float cy = data[1];
        float w  = data[2];
        float h  = data[3];

        cv::Mat scores(1, static_cast<int>(class_names_.size()), CV_32F, (void*)(data + 4));

        cv::Point class_id_point;
        double max_class_score;
        cv::minMaxLoc(scores, nullptr, &max_class_score, nullptr, &class_id_point);

        float confidence = static_cast<float>(max_class_score);
        if (confidence < score_threshold_) continue;

        int class_id = class_id_point.x;
        if (class_id < 0 || class_id >= static_cast<int>(class_names_.size())) continue;

        const std::string& class_name = class_names_[class_id];
        if (real_heights_.find(class_name) == real_heights_.end()) continue;

        float x1 = (cx - 0.5f * w - static_cast<float>(pad_w)) / scale;
        float y1 = (cy - 0.5f * h - static_cast<float>(pad_h)) / scale;
        float x2 = (cx + 0.5f * w - static_cast<float>(pad_w)) / scale;
        float y2 = (cy + 0.5f * h - static_cast<float>(pad_h)) / scale;

        int left   = std::max(0, std::min(static_cast<int>(std::round(x1)), frame.cols - 1));
        int top    = std::max(0, std::min(static_cast<int>(std::round(y1)), frame.rows - 1));
        int right  = std::max(0, std::min(static_cast<int>(std::round(x2)), frame.cols - 1));
        int bottom = std::max(0, std::min(static_cast<int>(std::round(y2)), frame.rows - 1));

        int bw = right - left;
        int bh = bottom - top;

        if (bw < 5 || bh < 20) continue;
        if (confidence < conf_threshold_) continue;

        boxes.emplace_back(left, top, bw, bh);
        confidences.push_back(confidence);
        class_ids.push_back(class_id);
    }

    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, conf_threshold_, nms_threshold_, indices);

    for (int idx : indices) {
        detections.push_back({boxes[idx], class_ids[idx], confidences[idx]});
    }

    return detections;
}

float DistanceDetector::estimateDistance(float real_height, float pixel_height) const
{
    if (pixel_height <= 1e-3f) return -1.0f;
    return (focal_length_ * real_height) / pixel_height;
}

float DistanceDetector::smoothPixelHeight(const std::string& class_name, float pixel_height)
{
    auto& buffer = pixel_buffers_[class_name];
    buffer.push_back(pixel_height);

    if (buffer.size() > 5) {
        buffer.pop_front();
    }

    std::vector<float> values(buffer.begin(), buffer.end());
    std::sort(values.begin(), values.end());

    return values[values.size() / 2];
}

void DistanceDetector::detectAndDraw(cv::Mat& frame)
{
    if (frame.empty()) return;

    float scale = 1.0f;
    int pad_w = 0;
    int pad_h = 0;

    cv::Mat blob = preprocess(frame, scale, pad_w, pad_h);
    net_.setInput(blob);

    std::vector<cv::Mat> outputs;
    net_.forward(outputs, net_.getUnconnectedOutLayersNames());

    std::vector<Detection> detections = postprocess(frame, outputs, scale, pad_w, pad_h);

    for (const auto& det : detections) {
        const std::string& class_name = class_names_[det.class_id];

        auto it = real_heights_.find(class_name);
        if (it == real_heights_.end()) continue;

        float pixel_height = static_cast<float>(det.box.height);
        pixel_height = smoothPixelHeight(class_name, pixel_height);

        float distance_m = estimateDistance(it->second, pixel_height);
        if (distance_m <= 0.0f) continue;

        cv::rectangle(frame, det.box, cv::Scalar(0, 255, 0), 2);

        std::string label = class_name + " " + cv::format("%.2f m", distance_m);

        int baseline = 0;
        cv::Size text_size = cv::getTextSize(label,
                                             cv::FONT_HERSHEY_SIMPLEX,
                                             0.55,
                                             2,
                                             &baseline);

        int tx = det.box.x;
        int ty = std::max(det.box.y - 8, text_size.height + 8);

        cv::rectangle(frame,
                      cv::Point(tx, ty - text_size.height - 8),
                      cv::Point(tx + text_size.width + 6, ty),
                      cv::Scalar(0, 255, 0),
                      cv::FILLED);

        cv::putText(frame,
                    label,
                    cv::Point(tx + 3, ty - 3),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.55,
                    cv::Scalar(0, 0, 0),
                    2);
    }
}