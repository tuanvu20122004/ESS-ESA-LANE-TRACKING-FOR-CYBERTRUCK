#include "LaneDetector.hpp"

LaneDetector::LaneDetector(const std::string& videoPath, int width, int height)
    : width(width), height(height) {
    if (videoPath.find("/dev/") != std::string::npos) {
        int capture_w = 1280, capture_h = 720;
        int output_w = 640, output_h = 480;
        int framerate = 30;

        std::string pipeline =
            "libcamerasrc ! "
            "video/x-raw, width=" + std::to_string(capture_w) +
            ", height=" + std::to_string(capture_h) +
            ", framerate=" + std::to_string(framerate) +
            "/1, format=(string)NV12 ! "
            "videoconvert ! "
            "video/x-raw, width=" + std::to_string(output_w) +
            ", height=" + std::to_string(output_h) +
            ", format=(string)BGRx ! "
            "videoconvert ! video/x-raw, format=(string)BGR ! appsink";

        std::cout << "[CAMERA] Using GStreamer pipeline:\n" << pipeline << "\n";
        cap.open(pipeline, cv::CAP_GSTREAMER);
    } else {
        // Nếu không phải /dev/, thì mở file video
        cap.open(videoPath);
    }

    // Kiểm tra sau khi đã mở (đúng logic)
    if (!cap.isOpened()) {
        std::cerr << "Cannot open: " << videoPath << std::endl;
        exit(-1);
    }
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

    // 1️⃣ Bird-eye view
    cv::Mat bird_eye_view = applyIPM(frame_resize);

    // 2️⃣ Mask theo màu trắng (lane)
    cv::Mat mask = processMask(bird_eye_view);

    // 3️⃣ Tìm điểm làn
    std::vector<cv::Point> left_points, right_points;
    cv::Vec3f left_coeffs(0,0,0), right_coeffs(0,0,0);
    bool left_ok = false, right_ok = false;

    slidingWindow(mask, left_points, right_points, bird_eye_view);
    left_ok  = (left_points.size()  >= 80);
    right_ok = (right_points.size() >= 80);

    cv::Vec3f temp_right_coeffs(0,0,0);
    if (right_ok) {
        if (left_ok)
            left_coeffs  = fitPoly(left_points, bird_eye_view, true);
        temp_right_coeffs = fitPoly(right_points, bird_eye_view, false);
        right_coeffs = temp_right_coeffs;
    }

    // 4️⃣ Phát hiện đổi làn
    bool change_lane = false;
    if (right_ok) {
        int mid_y = bird_eye_view.rows;
        float slope_right = computeLaneSlope(right_coeffs, mid_y);
        float slope_threshold = 0.3f;
        change_lane = (std::abs(slope_right) > slope_threshold);
    }

    // 5️⃣ Xử lý đổi làn nếu có
    if (change_lane) {
        right_points.clear();
        slidingWindowAdaptive(mask, right_points, bird_eye_view, temp_right_coeffs);
        right_ok = (right_points.size() >= 80);
        if (right_ok)
            right_coeffs = fitPoly(right_points, bird_eye_view, false);
        else
            right_coeffs = cv::Vec3f(0,0,0);

        left_ok = false;
        left_coeffs = cv::Vec3f(0,0,0);
    }

    // 6️⃣ Tính centerline
    std::vector<cv::Point> centerline = computeCenterline(left_coeffs, right_coeffs,
                                                          left_ok, right_ok, bird_eye_view);

    // 7️⃣ Tính thông số MPC
    current_state_ = mpc_computer_.computeMpcParameters(centerline, bird_eye_view);

    // 8️⃣ Hiển thị kết quả
    cv::Rect textBox(10, 10, 400, 120);
    cv::rectangle(frame_resize, textBox, cv::Scalar(0, 0, 0), -1);
    cv::rectangle(frame_resize, textBox, cv::Scalar(0, 255, 255), 2);

    if (current_state_.is_valid) {
        cv::putText(frame_resize, "Curvature: " + std::to_string(current_state_.curvature[0]),
                    {20, 35}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {0,255,255}, 1);
        cv::putText(frame_resize, "Offset: " + std::to_string(current_state_.lateral_deviation),
                    {20, 60}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {0,255,255}, 1);
        float yaw_deg = current_state_.yaw_angle * 180.0f / M_PI;
        cv::putText(frame_resize, "Yaw: " + std::to_string(yaw_deg) + " deg",
                    {20, 85}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {0,255,255}, 1);
    } else {
        cv::putText(frame_resize, "MPC: INVALID", {20, 60},
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, {0,0,255}, 1);
    }

    std::string status = "Status: ";
    if (left_ok && right_ok) status += "Both Lanes";
    else if (left_ok) status += "Left Only";
    else if (right_ok) status += "Right Only";
    else status += "No Lane";
    cv::putText(frame_resize, status, {20, 110}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {0,255,0}, 1);

    //cv::imshow("Mask", mask);
    //cv::imshow("Bird's-eye View", bird_eye_view);
    //cv::imshow("Lane Detection", frame_resize);
}

// ================= Core functions =================

cv::Mat LaneDetector::applyIPM(cv::Mat& frame) {
    float offsetX = 80.0f, offsetY = 0.0f;

    cv::Point2f tl(width * 0.20f + offsetX, height * 0.65f + offsetY);
    cv::Point2f bl(0.0f + offsetX, height + offsetY);
    cv::Point2f tr(width * 0.80f - offsetX, height * 0.65f + offsetY);
    cv::Point2f br(width - offsetX, height + offsetY);

    std::vector<cv::Point2f> src = { tl, bl, tr, br };
    std::vector<cv::Point2f> dst = {
        {0, 0}, {0, (float)height}, {(float)width, 0}, {(float)width, (float)height}
    };

    cv::Mat M = cv::getPerspectiveTransform(src, dst);
    cv::Mat bird_eye;
    cv::warpPerspective(frame, bird_eye, M, cv::Size(width, height));
    return bird_eye;
}

cv::Mat LaneDetector::processMask(const cv::Mat& bird_eye_view) {
    cv::Mat hsv, mask;
    cv::cvtColor(bird_eye_view, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(0, 0, 200), cv::Scalar(180, 40, 255), mask);
    return mask;
}

void LaneDetector::slidingWindow(const cv::Mat& mask, std::vector<cv::Point>& left_points, std::vector<cv::Point>& right_points, cv::Mat& outImg) {
    int nwindows = 15, margin = 60, minpix = 50;
    int height = mask.rows, width = mask.cols;
    int window_height = height / nwindows;

    cv::Mat hist;
    cv::reduce(mask(cv::Rect(0, height/2, width, height/2)),hist, 0, cv::REDUCE_SUM, CV_32S);
    int midpoint = hist.cols / 2;
    int leftx_base = std::max_element(hist.begin<int>(), hist.begin<int>() + midpoint) - hist.begin<int>();
    int rightx_base = std::max_element(hist.begin<int>() + midpoint, hist.end<int>()) - hist.begin<int>();

    int leftx_current = leftx_base, rightx_current = rightx_base;

    std::vector<cv::Point> nonzero;
    cv::findNonZero(mask, nonzero);

    for (int w = 0; w < nwindows; w++) {
        int win_y_low = height - (w+1) * window_height;
        cv::Rect left_win(leftx_current - margin, win_y_low, margin*2, window_height);
        cv::Rect right_win(rightx_current - margin, win_y_low, margin*2, window_height);

        std::vector<cv::Point> good_left, good_right;
        for (auto& p : nonzero) {
            if (left_win.contains(p))  good_left.push_back(p);
            if (right_win.contains(p)) good_right.push_back(p);
        }

        if ((int)good_left.size() > minpix)
            leftx_current = std::accumulate(good_left.begin(), good_left.end(), 0, [](int sum, cv::Point p){ return sum + p.x; }) / good_left.size();

        if ((int)good_right.size() > minpix)
            rightx_current = std::accumulate(good_right.begin(), good_right.end(), 0, [](int sum, cv::Point p){ return sum + p.x; }) / good_right.size();

        left_points.insert(left_points.end(), good_left.begin(), good_left.end());
        right_points.insert(right_points.end(), good_right.begin(), good_right.end());

        cv::rectangle(outImg, left_win, {0,255,0}, 2);
        cv::rectangle(outImg, right_win, {0,255,0}, 2);
    }
}

// ================= Advanced functions =================

void LaneDetector::slidingWindowAdaptive(const cv::Mat& mask, std::vector<cv::Point>& lane_points, cv::Mat& outImg, cv::Vec3f prev_poly) {
    int nwindows = 15, margin = 60;
    int height = mask.rows;
    int window_height = height / nwindows;

    std::vector<cv::Point> nonzero;
    cv::findNonZero(mask, nonzero);

    for (int window = 0; window < nwindows; window++) {
        int win_y_low = height - (window+1) * window_height;
        int y_mid = win_y_low + window_height / 2;
        int x_center = (int)(prev_poly[0]*y_mid*y_mid + prev_poly[1]*y_mid + prev_poly[2]);

        cv::Rect win(x_center - margin, win_y_low, margin*2, window_height);
        win &= cv::Rect(0, 0, mask.cols, mask.rows);

        cv::rectangle(outImg, win, cv::Scalar(255,0,0), 2);
        for (auto& p : nonzero)
            if (win.contains(p)) lane_points.push_back(p);
    }
}

std::vector<std::vector<cv::Point>> LaneDetector::findContoursInMask(const cv::Mat& mask) {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    return contours;
}

cv::Vec3f LaneDetector::fitPoly(const std::vector<cv::Point>& points, cv::Mat& outImg, bool isLeft) {
    if (points.size() < 3) return cv::Vec3f(0,0,0);

    std::vector<float> x, y;
    for (auto& p : points) { x.push_back(p.x); y.push_back(p.y); }

    cv::Mat Y(y.size(), 1, CV_32F, y.data());
    cv::Mat X(x.size(), 1, CV_32F, x.data());
    cv::Mat A(Y.rows, 3, CV_32F);
    for (int i = 0; i < Y.rows; ++i) {
        float yy = Y.at<float>(i);
        A.at<float>(i,0)=yy*yy; A.at<float>(i,1)=yy; A.at<float>(i,2)=1.0f;
    }
    cv::Mat coeffs;
    bool ok = cv::solve(A,X,coeffs,cv::DECOMP_SVD);
    if (!ok) return cv::Vec3f(0,0,0);

    cv::Vec3f c(coeffs.at<float>(0), coeffs.at<float>(1), coeffs.at<float>(2));
    cv::Scalar color = isLeft ? cv::Scalar(0,255,0) : cv::Scalar(0,0,255);
    for (int yv = 0; yv < outImg.rows; yv+=10) {
        int xv = (int)(c[0]*yv*yv + c[1]*yv + c[2]);
        if (xv>=0 && xv<outImg.cols)
            cv::circle(outImg, {xv,yv}, 2, color, -1);
    }
    return c;
}

std::vector<cv::Point> LaneDetector::computeCenterline(cv::Vec3f coeff_left, cv::Vec3f coeff_right, bool has_left, bool has_right, cv::Mat& outImg) {
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
    return 2*coeffs[0]*y + coeffs[1];
}
