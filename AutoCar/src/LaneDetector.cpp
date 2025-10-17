#include "LaneDetector.hpp"

LaneDetector::LaneDetector(const std::string& videoPath, int width, int height) 
    : current_steering_cmd_(0.0f), current_servo_angle_(93), has_steering_info_(false) {
    cap.open(videoPath);
    if (!cap.isOpened()) {
        std::cout << "Không mở được video!" << std::endl;
        exit(-1);
    }
    this->width = width;
    this->height = height;
}

LaneDetector::~LaneDetector() {
    cap.release();
    cv::destroyAllWindows();
}

bool LaneDetector::getFrame(cv::Mat& frame) {
    cap >> frame;
    return !frame.empty();
}

bool LaneDetector::isOpened() const {
    return cap.isOpened();
}

void LaneDetector::processFrame(cv::Mat& frame) {
    cv::Mat frame_resize;
    cv::resize(frame, frame_resize, cv::Size(width, height));
    
    cv::Mat bird_eye_view = applyIPM(frame_resize);
    cv::Mat mask = processMask(bird_eye_view);

    std::vector<cv::Point> left_points, right_points;
    cv::Vec3f left_coeffs(0,0,0), right_coeffs(0,0,0);
    bool left_ok = false, right_ok = false;

    slidingWindow(mask, left_points, right_points, bird_eye_view);
    left_ok  = (left_points.size()  >= 80);
    right_ok = (right_points.size() >= 80);

    cv::Vec3f temp_right_coeffs(0,0,0);
    if (right_ok) {
        if (left_ok) {
            left_coeffs  = fitPoly(left_points, bird_eye_view, true);
        }
        temp_right_coeffs = fitPoly(right_points, bird_eye_view, false);
        right_coeffs = temp_right_coeffs; 
    }

    bool change_lane = false;
    if (right_ok) {
        int mid_y = bird_eye_view.rows;
        float slope_right = computeLaneSlope(right_coeffs, mid_y);
        float slope_threshold = 0.3f;
        change_lane = (std::abs(slope_right) > slope_threshold);
    }

    if (change_lane) {                
        right_points.clear();
        slidingWindowAdaptive(mask, right_points, bird_eye_view, temp_right_coeffs); 
        right_ok = (right_points.size() >= 80);
        if (right_ok) {
            right_coeffs  = fitPoly(right_points, bird_eye_view, false);
        } else {
            right_coeffs = cv::Vec3f(0,0,0);
        }
        left_ok = false;
        left_coeffs = cv::Vec3f(0,0,0);
    }

    // Tính centerline
    std::vector<cv::Point> centerline = computeCenterline(left_coeffs, right_coeffs,
                                                           left_ok, right_ok,
                                                           bird_eye_view);

    // ========== TÍNH CÁC THAM SỐ MPC và LƯU VÀO current_state_ ==========
    current_state_ = mpc_computer_.computeMpcParameters(centerline, bird_eye_view);

    // ========== HIỂN THỊ LÊN FRAME ==========
    // Tạo một vùng nền đen cho text (tăng chiều cao để chứa thêm thông tin)
    cv::Rect textBox(10, 10, 400, 180);
    cv::rectangle(frame_resize, textBox, cv::Scalar(0, 0, 0), -1);
    cv::rectangle(frame_resize, textBox, cv::Scalar(0, 255, 255), 2);

    if (current_state_.is_valid) {
        // Hiển thị curvature (phần tử đầu tiên)
        std::string text_curv = "Curvature: " + 
            std::to_string(current_state_.curvature[0]) + " (1/m)";
        cv::putText(frame_resize, text_curv, 
                   cv::Point(20, 35), 
                   cv::FONT_HERSHEY_SIMPLEX, 
                   0.5, cv::Scalar(0, 255, 255), 1);

        // Hiển thị offset
        std::string text_lat = "Offset: " + 
            std::to_string(current_state_.lateral_deviation) + " (m)";
        cv::putText(frame_resize, text_lat, 
                   cv::Point(20, 60), 
                   cv::FONT_HERSHEY_SIMPLEX, 
                   0.5, cv::Scalar(0, 255, 255), 1);

        // Hiển thị angle_y
        float yaw_degree = current_state_.yaw_angle * 180.0f / M_PI;
        std::string text_yaw = "Angle_y: " + std::to_string(yaw_degree) + " (deg)";
        cv::putText(frame_resize, text_yaw, 
                   cv::Point(20, 85), 
                   cv::FONT_HERSHEY_SIMPLEX, 
                   0.5, cv::Scalar(0, 255, 255), 1);
        
        // Hiển thị steering command (góc lái từ MPC)
        if (has_steering_info_) {
            std::string text_cmd = "Steering CMD: " + 
                std::to_string(current_steering_cmd_) + " (deg)";
            cv::putText(frame_resize, text_cmd, 
                       cv::Point(20, 110), 
                       cv::FONT_HERSHEY_SIMPLEX, 
                       0.5, cv::Scalar(255, 128, 0), 1);
            
            // Hiển thị servo angle
            std::string text_servo = "Servo Angle: " + 
                std::to_string(current_servo_angle_);
            cv::putText(frame_resize, text_servo, 
                       cv::Point(20, 135), 
                       cv::FONT_HERSHEY_SIMPLEX, 
                       0.5, cv::Scalar(255, 128, 0), 1);
        }
        
    } else {
        cv::putText(frame_resize, "MPC: INVALID", 
                   cv::Point(20, 60), 
                   cv::FONT_HERSHEY_SIMPLEX, 
                   0.5, cv::Scalar(0, 0, 255), 1);
    }

    // Hiển thị status
    std::string status = "Status: ";
    if (left_ok && right_ok) status += "Both Lanes";
    else if (left_ok) status += "Left Only";
    else if (right_ok) status += "Right Only";
    else status += "No Lane";
    
    cv::putText(frame_resize, status, 
               cv::Point(20, 160), 
               cv::FONT_HERSHEY_SIMPLEX, 
               0.5, cv::Scalar(0, 255, 0), 1);

    // ========== HIỂN THỊ CÁC CỬA SỔ ==========
    cv::imshow("Mask", mask);
    cv::imshow("Lane Detection", frame_resize);
    cv::imshow("Bird's-eye View", bird_eye_view);
}

cv::Mat LaneDetector::applyIPM(cv::Mat& frame) {
    float offsetY = 0.0f;
    float offsetX = 80.0f;

    cv::Point2f tl(width * 0.20f + offsetX, height * 0.65f + offsetY);
    cv::Point2f bl(0.0f   + offsetX,        height      + offsetY);
    cv::Point2f tr(width * 0.80f - offsetX, height * 0.65f + offsetY);
    cv::Point2f br(width  - offsetX,        height      + offsetY);

    std::vector<cv::Point2f> src_points = { tl, bl, tr, br };

    for (size_t i = 0; i < src_points.size(); i++) {
        cv::circle(frame, src_points[i], 5, cv::Scalar(0, 255, 0), -1);
    }

    std::vector<cv::Point2f> dst_points = {
        cv::Point2f(0, 0),
        cv::Point2f(0, height),
        cv::Point2f(width, 0),
        cv::Point2f(width, height)
    };

    cv::Mat M = cv::getPerspectiveTransform(src_points, dst_points);
    cv::Mat bird_eye_view;
    cv::warpPerspective(frame, bird_eye_view, M, cv::Size(width, height));
    return bird_eye_view;
}

void LaneDetector::slidingWindow(const cv::Mat& mask,
                       std::vector<cv::Point>& left_points,
                       std::vector<cv::Point>& right_points,
                       cv::Mat& outImg) {
    int nwindows = 15;
    int margin   = 60;
    int minpix   = 50;
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

std::vector<std::vector<cv::Point>> LaneDetector::findContoursInMask(const cv::Mat& mask) {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    return contours;
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

    float cm_per_px = 0.02f;
    float laneW_nominal = 35.0f / cm_per_px;

    static float laneW_avg = laneW_nominal;
    const float alpha = 0.2f;

    auto evalX = [](cv::Vec3f c, float y){ return c[0]*y*y + c[1]*y + c[2]; };

    if (has_left && has_right) {
        std::vector<float> widths;
        for (int y=0; y<outImg.rows; y+=20)
            widths.push_back(fabs(evalX(coeff_right,y)-evalX(coeff_left,y)));
        std::nth_element(widths.begin(), widths.begin()+widths.size()/2, widths.end());
        float median_w = widths[widths.size()/2];
        laneW_avg = (1-alpha)*laneW_avg + alpha*median_w;
    }

    for (int y=0; y<outImg.rows; y+=10) {
        float xc=-1;
        if (has_left && has_right)
            xc = 0.5f*(evalX(coeff_left,y)+evalX(coeff_right,y));
        else if (has_left)
            xc = evalX(coeff_left,y) + 0.5f*laneW_avg;
        else if (has_right)
            xc = evalX(coeff_right,y) - 0.5f*laneW_avg;
        else continue;

        int x = std::clamp((int)std::round(xc),0,outImg.cols-1);
        centerline.push_back({x,y});
        cv::circle(outImg,{x,y},2,{255,255,0},-1);
    }
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
