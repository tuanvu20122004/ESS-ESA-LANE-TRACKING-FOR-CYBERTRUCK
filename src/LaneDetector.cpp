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
    // ======== 1️Bird-eye & mask =========
    bird_eye_view_ = applyIPM(frame_resize);
    mask = processMask(bird_eye_view_);

    std::vector<cv::Point> left_points, right_points;
    cv::Vec3f left_coeffs(0, 0, 0), right_coeffs(0, 0, 0);
    bool left_ok = false, right_ok = false;

    // ======== 2️Phát hiện lane =========
    slidingWindow(mask, left_points, right_points, bird_eye_view_);

    left_ok  = (left_points.size()  >= 120);
    right_ok = (right_points.size() >= 120);

    if (left_ok)  left_coeffs  = fitPoly(left_points, bird_eye_view_, true);
    if (right_ok) right_coeffs = fitPoly(right_points, bird_eye_view_, false);

    // ======== 3️Tạo đường trung tâm theo tình huống =========
    std::vector<cv::Point> centerline;
    float lane_offset_px = (31.0f / 0.02f) / 2.0f; // 31cm ~ 1750px (tùy calib)

    if (left_ok && right_ok) {
        // Có 2 làn → lấy trung bình
        centerline = computeCenterline(left_coeffs, right_coeffs, true, true, bird_eye_view_);
    } 
    else if (right_ok && !left_ok) {
        // Chỉ có làn phải → dịch sang trái
        std::vector<cv::Point> center_pts;
        for (int y = 0; y < bird_eye_view_.rows; y += 10) {
            float xr = right_coeffs[0]*y*y + right_coeffs[1]*y + right_coeffs[2];
            float slope = 2*right_coeffs[0]*y + right_coeffs[1];
            float theta = std::atan(slope);
            float x_center = xr - std::cos(theta - CV_PI/2) * lane_offset_px;
            int ix = std::clamp((int)std::round(x_center), 0, bird_eye_view_.cols-1);
            center_pts.push_back({ix, y});
        }
        centerline = center_pts;
    } 
    else if (left_ok && !right_ok) {
        // Chỉ có làn trái → dịch sang phải
        std::vector<cv::Point> center_pts;
        for (int y = 0; y < bird_eye_view_.rows; y += 10) {
            float xl = left_coeffs[0]*y*y + left_coeffs[1]*y + left_coeffs[2];
            float slope = 2*left_coeffs[0]*y + left_coeffs[1];
            float theta = std::atan(slope);
            float x_center = xl + std::cos(theta + CV_PI/2) * lane_offset_px;
            int ix = std::clamp((int)std::round(x_center), 0, bird_eye_view_.cols-1);
            center_pts.push_back({ix, y});
        }
        centerline = center_pts;
    } 
    else {
        // Không có làn nào
        has_valid_lane_ = false;
        //displayInfo(frame_resize, false, false);
        return;
    }

    // ======== 4 Xử lý đặc biệt khúc cong =========
    if (left_ok && right_ok) {
        float y_eval = bird_eye_view_.rows;
        float slope_left = computeLaneSlope(left_coeffs, y_eval);
        float slope_right = computeLaneSlope(right_coeffs, y_eval);
        float diff = std::fabs(slope_left - slope_right);
        if (diff > 0.5f) {
            // Hai làn lệch mạnh nhau -> chỉ giữ làn tốt hơn
            if (std::fabs(slope_right) < std::fabs(slope_left))
                left_ok = false;
            else
                right_ok = false;
        }
    }

    // ======== 5️Cập nhật dữ liệu lane =========
    centerline_ = centerline;
    has_valid_lane_ = (centerline_.size() >= 3);

    // ======== 6️Hiển thị overlay =========
    //displayInfo(frame_resize, left_ok, right_ok);
}
/*void LaneDetector::displayInfo(cv::Mat& frame_resize, bool left_ok, bool right_ok) {
    cv::Rect textBox(10, 10, 400, 180);
    cv::rectangle(frame_resize, textBox, cv::Scalar(0, 0, 0), -1);
    cv::rectangle(frame_resize, textBox, cv::Scalar(0, 255, 255), 2);

    if (has_mpc_data_) {
        std::string text_curv = "Curvature: " + 
            std::to_string(display_curvature_) + " (1/m)";
        cv::putText(frame_resize, text_curv, 
                   cv::Point(20, 35), 
                   cv::FONT_HERSHEY_SIMPLEX, 
                   0.5, cv::Scalar(0, 255, 255), 1);

        std::string text_lat = "Offset: " + 
            std::to_string(display_lateral_dev_) + " (m)";
        cv::putText(frame_resize, text_lat, 
                   cv::Point(20, 60), 
                   cv::FONT_HERSHEY_SIMPLEX, 
                   0.5, cv::Scalar(0, 255, 255), 1);

        float yaw_degree = display_yaw_angle_ * 180.0f / M_PI;
        std::string text_yaw = "Angle_y: " + std::to_string(yaw_degree) + " (deg)";
        cv::putText(frame_resize, text_yaw, 
                   cv::Point(20, 85), 
                   cv::FONT_HERSHEY_SIMPLEX, 
                   0.5, cv::Scalar(0, 255, 255), 1);
        
        if (has_steering_info_) {
            std::string text_cmd = "Steering CMD: " + 
                std::to_string(current_steering_cmd_) + " (deg)";
            cv::putText(frame_resize, text_cmd, 
                       cv::Point(20, 110), 
                       cv::FONT_HERSHEY_SIMPLEX, 
                       0.5, cv::Scalar(255, 128, 0), 1);
            
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

    std::string status = "Status: ";
    if (left_ok && right_ok) status += "Both Lanes";
    else if (left_ok) status += "Left Only";
    else if (right_ok) status += "Right Only";
    else status += "No Lane";
    
    cv::putText(frame_resize, status, 
               cv::Point(20, 160), 
               cv::FONT_HERSHEY_SIMPLEX, 
               0.5, cv::Scalar(0, 255, 0), 1);
}*/

cv::Mat LaneDetector::applyIPM(cv::Mat& frame) {
    float offsetY = -70.0f;
    float offsetX = 100.0f;

    cv::Point2f tl(width * 0.20f + offsetX, height * 0.65f + offsetY);
    cv::Point2f bl(32.0f   + offsetX, height - 140);
    cv::Point2f tr(width * 0.85f - offsetX, height * 0.65f + offsetY);
    cv::Point2f br(width  - offsetX, height - 140);

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

/*void LaneDetector::slidingWindow(const cv::Mat& mask,
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
}*/
void LaneDetector::slidingWindow(const cv::Mat& mask,
                                 std::vector<cv::Point>& left_points,
                                 std::vector<cv::Point>& right_points,
                                 cv::Mat& outImg) {
    // ==== Tham số (tinh chỉnh theo dữ liệu thực) ====
    const int nwindows   = 15;
    const int margin     = 50;   // hẹp hơn để giảm chồng lấn
    const int minpix     = 50;   // số điểm tối thiểu để dời tâm mỗi cửa sổ
    const int peak_points_per_col = 40; // ngưỡng đỉnh histogram (điểm/ cột)
    const float min_sep_after_fit = 120.f; // (dùng ở bước sau-fit nếu muốn)

    left_points.clear();
    right_points.clear();
    if (mask.empty()) return;

    const int H = mask.rows;
    const int W = mask.cols;
    const int win_h = std::max(1, H / nwindows);
    const int midx = W / 2;

    // ==== 1) Histogram nửa dưới ảnh: tìm đỉnh trái/phải ====
    cv::Mat hist; // 1 x W, kiểu CV_32S
    cv::reduce(mask(cv::Rect(0, H/2, W, H/2)), hist, 0, cv::REDUCE_SUM, CV_32S);

    cv::Mat leftHist  = hist.colRange(0, midx);
    cv::Mat rightHist = hist.colRange(midx, W);

    double lMax = 0, rMax = 0;
    cv::Point lLoc, rLoc;
    cv::minMaxLoc(leftHist,  nullptr, &lMax, nullptr, &lLoc);
    cv::minMaxLoc(rightHist, nullptr, &rMax, nullptr, &rLoc);

    // mỗi pixel mask = 255 -> quy đổi sang số điểm/ cột
    const double PEAK_THR = 255.0 * peak_points_per_col;

    bool track_left  = (lMax >= PEAK_THR);
    bool track_right = (rMax >= PEAK_THR);

    int leftx_current  = track_left  ? lLoc.x          : -1;
    int rightx_current = track_right ? (midx + rLoc.x) : -1;

    // Ràng buộc vùng hợp lệ của tâm cửa sổ theo nửa ảnh
    auto clamp_left_x  = [&](int x){ return std::clamp(x, margin,            midx - 1 - margin); };
    auto clamp_right_x = [&](int x){ return std::clamp(x, midx + margin,     W - 1 - margin);   };

    if (track_left)  leftx_current  = clamp_left_x(leftx_current);
    if (track_right) rightx_current = clamp_right_x(rightx_current);

    // Lấy tất cả điểm khác 0 để quét nhanh theo cửa sổ
    std::vector<cv::Point> nz;
    cv::findNonZero(mask, nz);

    // ==== 2) Cửa sổ trượt từ dưới lên ====
    for (int w = 0; w < nwindows; ++w) {
        const int y_low  = H - (w + 1) * win_h;
        const int y_high = H - w * win_h;

        cv::Rect left_win, right_win;
        bool haveL = false, haveR = false;

        if (track_left) {
            left_win = cv::Rect(leftx_current - margin, y_low, margin * 2, win_h);
            left_win &= cv::Rect(0, 0, W, H);
            haveL = (left_win.area() > 0);
        }
        if (track_right) {
            right_win = cv::Rect(rightx_current - margin, y_low, margin * 2, win_h);
            right_win &= cv::Rect(0, 0, W, H);
            haveR = (right_win.area() > 0);
        }

        std::vector<cv::Point> good_left, good_right;

        // ==== Nhặt điểm với "độc quyền" (không để điểm rơi vào cả hai bên) ====
        for (const auto& p : nz) {
            if (p.y < y_low || p.y >= y_high) continue;

            bool inL = haveL && left_win.contains(p);
            bool inR = haveR && right_win.contains(p);

            if (inL && !inR) {
                good_left.push_back(p);
            } else if (!inL && inR) {
                good_right.push_back(p);
            } else if (inL && inR) {
                // gán cho cửa sổ có tâm gần hơn theo trục x
                int dl = std::abs(p.x - leftx_current);
                int dr = std::abs(p.x - rightx_current);
                (dl <= dr ? good_left : good_right).push_back(p);
            }
        }

        // ==== Vẽ debug cửa sổ (tuỳ chọn) ====
        if (haveL && !good_left.empty())
            cv::rectangle(outImg, left_win,  cv::Scalar(128,128,128), 1); // xám để không nhầm với lane
        if (haveR && !good_right.empty())
            cv::rectangle(outImg, right_win, cv::Scalar(128,128,128), 1);

        // ==== Gom điểm ====
        left_points.insert(left_points.end(),   good_left.begin(),  good_left.end());
        right_points.insert(right_points.end(), good_right.begin(), good_right.end());

        // ==== Dời tâm theo trung bình x nếu đủ minpix ====
        if (haveL && static_cast<int>(good_left.size()) > minpix) {
            int sumx = 0; for (auto &p : good_left) sumx += p.x;
            leftx_current = clamp_left_x(sumx / static_cast<int>(good_left.size()));
        }
        if (haveR && static_cast<int>(good_right.size()) > minpix) {
            int sumx = 0; for (auto &p : good_right) sumx += p.x;
            rightx_current = clamp_right_x(sumx / static_cast<int>(good_right.size()));
        }
    }

    // (Tuỳ chọn) Bạn có thể thêm bước "lọc hậu-fit" ngay tại đây sau khi fitPoly,
    // kiểm tra khoảng cách trung vị giữa 2 đa thức và loại bỏ 1 bên nếu quá sát.
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

/*std::vector<cv::Point> LaneDetector::computeCenterline(cv::Vec3f coeff_left,
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
        std::cout << "[LaneDetector] Lane width updated: " << laneW_avg*cm_per_px << " cm\n";
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
}*/
std::vector<cv::Point> LaneDetector::computeCenterline(cv::Vec3f coeff_left,
                                            cv::Vec3f coeff_right,
                                            bool has_left, bool has_right,
                                            cv::Mat& outImg) {
    std::vector<cv::Point> centerline;
    if (outImg.empty()) return centerline;

    // ====== 1Thông số calib ======
    float cm_per_px = 0.02f;                // 1 pixel ≈ 2cm (tùy camera, cần calib lại)
    float laneW_nominal = 31.0f / cm_per_px; // 35cm ~ 1750px
    static float laneW_avg = laneW_nominal;
    const float alpha = 0.2f;               // hệ số cập nhật mượt (EMA)

    // ====== 2️Hàm nội suy vị trí x từ y ======
    auto evalX = [](cv::Vec3f c, float y) {
        return c[0] * y * y + c[1] * y + c[2];
    };

    // ====== 3️Cập nhật độ rộng làn trung bình (nếu có đủ 2 làn) ======
    if (has_left && has_right) {
        std::vector<float> widths;
        for (int y = 0; y < outImg.rows; y += 20)
            widths.push_back(std::fabs(evalX(coeff_right, y) - evalX(coeff_left, y)));

        if (!widths.empty()) {
            std::nth_element(widths.begin(), widths.begin() + widths.size() / 2, widths.end());
            float median_w = widths[widths.size() / 2];
            laneW_avg = (1 - alpha) * laneW_avg + alpha * median_w;
            if(laneW_avg < 0.95f * laneW_nominal || laneW_avg > 1.05f * laneW_nominal) {
                laneW_avg = laneW_nominal; // tránh sai số lớn
            }
            std::cout << "[LaneDetector] Lane width updated: " 
                      << laneW_avg * cm_per_px << " cm\n";
        }
    }

    // ====== 4️Tính centerline ======
    for (int y = 0; y < outImg.rows; y += 10) {
        float xc = -1.0f;

        if (has_left && has_right) {
            // Hai làn: lấy trung bình
            xc = 0.5f * (evalX(coeff_left, y) + evalX(coeff_right, y));
        } 
        else if (has_left) {
            // Chỉ làn trái: dịch theo pháp tuyến sang phải
            float x_left = evalX(coeff_left, y);
            float slope = 2 * coeff_left[0] * y + coeff_left[1];
            float theta = std::atan(slope);
            // Offset động dựa trên độ cong
            float curvature_factor = std::clamp(std::fabs(2 * coeff_left[0]), 0.0001f, 0.002f);
            float dynamic_offset = (laneW_avg * 0.6f) / (1.0f + 150.0f * curvature_factor);

            xc = x_left + std::cos(theta + CV_PI / 2) * dynamic_offset;
            //xc = x_left + std::cos(theta + CV_PI / 2) * (laneW_avg * 0.5f);
        } 
        else if (has_right) {
            // Chỉ làn phải: dịch theo pháp tuyến sang trái
            float x_right = evalX(coeff_right, y);
            float slope = 2 * coeff_right[0] * y + coeff_right[1];
            float theta = std::atan(slope);
            // Offset động dựa trên độ cong
            float curvature_factor = std::clamp(std::fabs(2 * coeff_right[0]), 0.0001f, 0.002f);
            float dynamic_offset = (laneW_avg * 0.6f) / (1.0f + 150.0f * curvature_factor);

            xc = x_right - std::cos(theta - CV_PI / 2) * dynamic_offset;
            //xc = x_right - std::cos(theta - CV_PI / 2) * (laneW_avg * 0.5f);
        } 
        else {
            continue;
        }

        int x = std::clamp(static_cast<int>(std::round(xc)), 0, outImg.cols - 1);
        centerline.push_back({x, y});

        cv::circle(outImg, {x, y}, 2, {255, 255, 0}, -1);
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