#include "LaneDetector.hpp"
#include <algorithm>
#include <iostream>

static bool try_open_gst(cv::VideoCapture& cap, const std::string& pipeline) {
    std::cout << "[CAMERA] Try pipeline:\n" << pipeline << "\n";
    if (!cap.open(pipeline, cv::CAP_GSTREAMER)) {
        std::cerr << "[CAMERA] Open failed.\n";
        return false;
    }
    std::cout << "[CAMERA] Opened OK.\n";
    return true;
}

LaneDetector::LaneDetector(const std::string& videoPath, int width, int height)
    : width(width), height(height)
{
    const int CAP_W = 640, CAP_H = 480;
    const int OUT_W = width, OUT_H = height;
    const int FPS   = 30;

    if (videoPath.find("/dev/") != std::string::npos) {
	// Chỉ dùng libcamera + videoscale
        const std::string p0 =
            "libcamerasrc ! "
            "video/x-raw, width=" + std::to_string(CAP_W) +
            ", height=" + std::to_string(CAP_H) +
            ", framerate=" + std::to_string(FPS) + "/1 ! "
            "videoconvert ! videoscale ! "
            "video/x-raw, format=(string)BGR, width=" + std::to_string(OUT_W) +
            ", height=" + std::to_string(OUT_H) + " ! "
            "appsink max-buffers=1 drop=true sync=false";

        const std::string p1 =
            "libcamerasrc ! "
            "videoconvert ! videoscale ! "
            "video/x-raw, format=(string)BGR, width=" + std::to_string(OUT_W) +
            ", height=" + std::to_string(OUT_H) + " ! "
            "appsink max-buffers=1 drop=true sync=false";

        if (!try_open_gst(cap, p0) && !try_open_gst(cap, p1)) {
            throw std::runtime_error("Cannot open camera with libcamerasrc: " + videoPath);
        }
    } else {
        if (!cap.open(videoPath)) {
            throw std::runtime_error("Cannot open file: " + videoPath);
        }
    }
}

LaneDetector::~LaneDetector() {
    cap.release();
    cv::destroyAllWindows();
}

bool LaneDetector::getFrame(cv::Mat& frame) {
    cap >> frame;
    cv::Mat undistorted;
    cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) <<
        262.08953333143063, 0.0, 330.77574325128484,
        0.0, 263.57901348164575, 250.50298224489268,
        0.0, 0.0, 1.0);
    cv::Mat distCoeffs = (cv::Mat_<double>(1,5) <<
        -0.27166331922859776, 0.09924985737514846,
        -0.0002707688044880526, 0.0006724194580262318,
        -0.01935517123682299);
    cv::undistort(frame, undistorted, cameraMatrix, distCoeffs);
    frame = undistorted.clone();

    cv::resize(frame, frame_resize, cv::Size(width, height));
    return !frame.empty();
}

bool LaneDetector::isOpened() const {
    return cap.isOpened();
}

cv::Mat LaneDetector::getFrameResize() {
    return frame_resize;
}
cv::Mat LaneDetector::getMask() const{
    return mask;
}
void LaneDetector::processFrame(cv::Mat& frame_resize) {
    bird_eye_view = applyIPM(frame_resize);
    mask = processMask(bird_eye_view);

    static int minpix=30;
    std::vector<cv::Point> left_points, right_points;
    cv::Vec3f left_coeffs(0,0,0), right_coeffs(0,0,0);
    static cv::Vec3f prev_left(0,0,0), prev_right(0,0,0);
    bool left_ok = false, right_ok = false;

    if (!initialized) {
        slidingWindow(mask, left_points, right_points, bird_eye_view, minpix);
        left_ok  = (left_points.size()  >= 80);
        right_ok = (right_points.size() >= 80);

        if (left_ok)  left_coeffs  = fitPoly(left_points, bird_eye_view, true);
        if (right_ok) right_coeffs = fitPoly(right_points, bird_eye_view, false);

        if (left_ok && right_ok) {
            initialized = true;
            prev_left = left_coeffs;
            prev_right = right_coeffs;
            std::cout << "Lane initialized" << std::endl;
        }
    } 
    else {
        slidingWindowAdaptive(mask, left_points, bird_eye_view, prev_left);
        slidingWindowAdaptive(mask, right_points, bird_eye_view, prev_right);

        if (left_points.size() >= 60) {
            left_ok = true;
            left_coeffs = fitPoly(left_points, bird_eye_view, true);
        } else {
            left_ok = false;
            left_coeffs = prev_left;
        }
        if (right_points.size() >= 60) {    
            right_ok = true;
            right_coeffs = fitPoly(right_points, bird_eye_view, false);
        } else {
            right_ok = false;
            right_coeffs = prev_right;
        }

        int mid_y = bird_eye_view.rows / 2;
        float slope_right = computeLaneSlope(right_coeffs, mid_y);
        float slope_left  = computeLaneSlope(left_coeffs, mid_y);
        bool change_lane = (std::fabs(slope_right) > 0.25f) || (std::fabs(slope_left) > 0.25f);

        if (change_lane) {
            std::cout << "Change lane detected - reinitialize!" << std::endl;
            initialized = false;  
            return;              
        }

        prev_left = left_coeffs;
        prev_right = right_coeffs;
    }

    centerline = computeCenterline(left_coeffs, right_coeffs, left_ok, right_ok, bird_eye_view);
    has_valid_lane_ = (centerline.size() >= 3);
}

cv::Mat LaneDetector::applyIPM(cv::Mat& frame) {
    float offsetY = -70.0f;
    float offsetX = 100.0f;

    cv::Point2f tl(width * 0.20f + offsetX, height * 0.65f + offsetY);
    cv::Point2f bl(32.0f   + offsetX, height - 140);
    cv::Point2f tr(width * 0.85f - offsetX, height * 0.65f + offsetY);
    cv::Point2f br(width  - offsetX, height - 140);
    
    std::vector<cv::Point2f> src_points = { tl, bl, tr, br };

    for (int i = 0; i < src_points.size(); i++) {
        cv::circle(frame, src_points[i], 5, cv::Scalar(0, 255, 0), -1);
    }

    std::vector<cv::Point2f> dst_points = {
        cv::Point2f(0, 0),
        cv::Point2f(0, height),
        cv::Point2f(width, 0),
        cv::Point2f(width, height)
    };

    cv::Mat M = cv::getPerspectiveTransform(src_points, dst_points);
    cv::warpPerspective(frame_resize, bird_eye_view, M, cv::Size(width, height));
    return bird_eye_view;
}

void LaneDetector::slidingWindow(const cv::Mat& mask,
                       std::vector<cv::Point>& left_points,
                       std::vector<cv::Point>& right_points,
                       cv::Mat& outImg, int minpix) {
    int nwindows = 15;
    int margin   = 60;
    int height   = mask.rows;
    int width    = mask.cols;
    int window_height = height / nwindows;

    cv::Mat hist;
    cv::reduce(mask(cv::Rect(0, height/2, width, height/2)),
            hist, 0, cv::REDUCE_SUM, CV_32S);
    int midpoint = hist.cols / 2;
    int leftx_base  = std::max_element(hist.begin<int>(), hist.begin<int>() + midpoint) - hist.begin<int>();
    int rightx_base = std::max_element(hist.begin<int>() + midpoint, hist.end<int>()) - hist.begin<int>();

    int leftx_current  = leftx_base;
    int rightx_current = rightx_base;

    std::vector<cv::Point> nonzero;
    cv::findNonZero(mask, nonzero);

    for (int window = 0; window < nwindows; window++) {
        int win_y_low  = height - (window+1) * window_height;

        cv::Rect left_win(leftx_current - margin, win_y_low, margin*2, window_height);
        cv::Rect right_win(rightx_current - margin, win_y_low, margin*2, window_height);

        std::vector<cv::Point> good_left, good_right;
        for (auto &p : nonzero) {
            if (left_win.contains(p))  good_left.push_back(p);
            if (right_win.contains(p)) good_right.push_back(p);
        }

        if (!good_left.empty()) {
            cv::rectangle(outImg, left_win, cv::Scalar(0,255,0), 2);
        }
        if (!good_right.empty()) {
            cv::rectangle(outImg, right_win, cv::Scalar(0,255,0), 2);
        }
        left_points.insert(left_points.end(), good_left.begin(), good_left.end());
        right_points.insert(right_points.end(), good_right.begin(), good_right.end());

        if ((int)good_left.size() > minpix) {
            int sumx = 0;
            for (auto &p : good_left) sumx += p.x;
            leftx_current = sumx / (int)good_left.size();
        }
        if ((int)good_right.size() > minpix) {
            int sumx = 0;
            for (auto &p : good_right) sumx += p.x;
            rightx_current = sumx / (int)good_right.size();
        }
    }
}

void LaneDetector::slidingWindowAdaptive(const cv::Mat& mask,
                               std::vector<cv::Point>& lane_points,
                               cv::Mat& outImg,
                               cv::Vec3f prev_poly) {
    int nwindows = 15;
    int margin   = 60;
    int height   = mask.rows;
    int window_height = height / nwindows;

    std::vector<cv::Point> nonzero;
    cv::findNonZero(mask, nonzero);

    int x_center = mask.cols / 2;

    for (int window = 0; window < nwindows; window++) {
        int win_y_low  = height - (window+1) * window_height;
        int win_y_high = height - window * window_height;
        int y_mid      = (win_y_low + win_y_high) / 2;

        x_center = (int)(prev_poly[0]*y_mid*y_mid +
                            prev_poly[1]*y_mid +
                            prev_poly[2]);

        cv::Rect win(x_center - margin, win_y_low, margin*2, window_height);
        win &= cv::Rect(0, 0, mask.cols, mask.rows);

        cv::rectangle(outImg, win, cv::Scalar(0,255,0), 2);

        for (auto &p : nonzero) {
            if (win.contains(p)) {
                lane_points.push_back(p);
            }
        }
    }
}

cv::Mat LaneDetector::processMask(const cv::Mat& bird_eye_view) {
    cv::Mat hsv, mask;
    cv::cvtColor(bird_eye_view, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(0, 0, 200), cv::Scalar(180, 40, 255), mask);
    return mask;
}

cv::Vec3f LaneDetector::fitPoly(const std::vector<cv::Point>& points, cv::Mat& outImg, bool isLeft) {
    if (points.size() < 2) return cv::Vec3f(0,0,0);
    std::vector<float> x_vals, y_vals;
    for (const auto& pt : points) {
        x_vals.push_back((float)pt.x);
        y_vals.push_back((float)pt.y);
    }
    cv::Mat Y(y_vals.size(), 1, CV_32F, y_vals.data());
    cv::Mat X(x_vals.size(), 1, CV_32F, x_vals.data());
    cv::Vec3f coeff_out(0,0,0);
    if (points.size() >= 3) {
        cv::Mat A2(Y.rows, 3, CV_32F);
        for (int i = 0; i < Y.rows; ++i) {
            float y = Y.at<float>(i, 0);
            A2.at<float>(i, 0) = y * y;
            A2.at<float>(i, 1) = y;
            A2.at<float>(i, 2) = 1.0f;
        }
        cv::Mat coeffs2;
        bool ok = cv::solve(A2, X, coeffs2, cv::DECOMP_SVD);
        if (ok) coeff_out = cv::Vec3f(coeffs2.at<float>(0), coeffs2.at<float>(1), coeffs2.at<float>(2));
    }
    if (coeff_out == cv::Vec3f(0,0,0) || std::fabs(coeff_out[0]) < 1e-6) {
        cv::Mat A1(Y.rows, 2, CV_32F);
        for (int i = 0; i < Y.rows; ++i) {
            float y = Y.at<float>(i, 0);
            A1.at<float>(i, 0) = y;
            A1.at<float>(i, 1) = 1.0f;
        }
        cv::Mat coeffs1;
        bool ok = cv::solve(A1, X, coeffs1, cv::DECOMP_SVD);
        if (ok) coeff_out = cv::Vec3f(0, coeffs1.at<float>(0), coeffs1.at<float>(1));
    }
    cv::Scalar lineColor = isLeft ? cv::Scalar(0,255,0) : cv::Scalar(0,0,255);
    if (coeff_out != cv::Vec3f(0,0,0)) {
        for (int y = 0; y < outImg.rows; ++y) {
            float x = coeff_out[0]*y*y + coeff_out[1]*y + coeff_out[2];
            int ix = (int)std::round(x);
            if (ix >= 0 && ix < outImg.cols) {
                cv::circle(outImg, cv::Point(ix, y), 2, lineColor, -1);
            }
        }
    }
    return coeff_out;
}

std::vector<cv::Point> LaneDetector::computeCenterline(cv::Vec3f coeff_left,
                                            cv::Vec3f coeff_right,
                                            bool has_left, bool has_right,
                                            cv::Mat& outImg) {
    std::vector<cv::Point> centerline;
    if (outImg.empty()) return centerline;

    const float LANE_WIDTH_PX = 310.0f;
    float laneW_avg = LANE_WIDTH_PX;  

    auto evalX = [](cv::Vec3f c, float y) {
        return c[0] * y * y + c[1] * y + c[2];
    };

    if (has_left && has_right) {
        std::vector<float> widths;
        float d;
        for (int y = 0; y < outImg.rows; y += 20) {
            d = fabs(evalX(coeff_right, y) - evalX(coeff_left, y));  
            if (d > 50 && d < 600)  
                widths.push_back(d);
        }
        if (!widths.empty()) {
            float sum =0;
            for(auto b: widths) sum += b;
            laneW_avg = sum/widths.size();
        }
    }
    laneW_avg = std::clamp(laneW_avg, 200.0f, 400.0f);  

    float current_laneW = laneW_avg;
    int midX = outImg.cols / 2;  

    for (int y = 0; y < outImg.rows; y += 10) {
        float xc = -1;
        if (has_left && has_right) {
            xc = 0.5f * (evalX(coeff_left, y) + evalX(coeff_right, y));  
        }

        else if (has_left) {
            xc = evalX(coeff_left, y) + 0.5f * current_laneW;  
        }

        else if (has_right) {
            xc = evalX(coeff_right, y) - 0.5f * current_laneW;
        } else {
            continue;  
        }
        int x = std::clamp((int)std::round(xc), 0, outImg.cols - 1);
        centerline.push_back({x, y});  
        cv::circle(outImg, {x, y}, 2, {255, 255, 0}, -1);
    }

    // ==== Debug hiển thị ==== 
    std::string dbg_text = "W=" + std::to_string((int)laneW_avg) + "px";
    cv::putText(outImg, dbg_text, {30, 40}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {0, 255, 255}, 2);

    // Hiển thị trạng thái lane
    if (has_left && !has_right)
        cv::putText(outImg, "LEFT ONLY", {30, 80}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {0, 255, 0}, 2);
    else if (!has_left && has_right)
        cv::putText(outImg, "RIGHT ONLY", {30, 80}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {0, 0, 255}, 2);
    else if (has_left && has_right)
        cv::putText(outImg, "BOTH LANES", {30, 80}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {255, 255, 255}, 2);
    else
        cv::putText(outImg, "NO LANE", {30, 80}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {0, 0, 255}, 2);

    return centerline; 
}

float LaneDetector::computeLaneSlope(const cv::Vec3f& coeffs, float y) {
    float a = coeffs[0];
    float b = coeffs[1];

    if (std::fabs(a) > 1e-6) {
        return 2*a*y + b;
    } else {
        return b;
    }
}
