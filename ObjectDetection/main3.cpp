#include <opencv2/opencv.hpp>
#include <iostream>

class LaneDetector {
private:
    cv::VideoCapture cap;
    int width, height;

public:
    LaneDetector(const std::string& videoPath, int width = 640, int height = 480) {
        cap.open(videoPath);
        if (!cap.isOpened()) {
            std::cout << "Không mở được video!" << std::endl;
            exit(-1);
        }
        this->width = width;
        this->height = height;
    }

    void processFrame() {
        cv::Mat frame, frame_resize;
        while (true) {
            cap >> frame;
            if (frame.empty()) break;

            cv::resize(frame, frame_resize, cv::Size(width, height));
            cv::Mat bird_eye_view = applyIPM(frame_resize);
            //cv::Mat roi = applyROI(bird_eye_view);
            cv::Mat mask = processMask(bird_eye_view);

            std::vector<cv::Point> left_points, right_points;
            cv::Vec3f left_coeffs, right_coeffs;
            bool left_ok = false, right_ok = false;

            // ===== 3. Nếu KHÔNG change_lane thì mới chạy sliding window =====

            slidingWindow(mask, left_points, right_points, bird_eye_view);
            left_coeffs  = fitPoly(left_points,  bird_eye_view, true);
            right_coeffs = fitPoly(right_points, bird_eye_view, false);

            left_ok  = (left_points.size()  >= 50);
            right_ok = (right_points.size() >= 50);

            bool change_lane = false;
            // ===== 4. Tính centerline =====
            computeCenterline(left_coeffs, right_coeffs,
                            left_ok, right_ok,
                            change_lane, bird_eye_view, mask);

            // ===== 5. Hiển thị =====
            cv::imshow("Mask (ROI)", mask);
            cv::imshow("Lane Detection", frame_resize);
            cv::imshow("Bird's-eye View", bird_eye_view);

            if (cv::waitKey(30) == 27) break; // ESC để thoát
        }
    }

    ~LaneDetector() {
        cap.release();
        cv::destroyAllWindows();
    }

    cv::Mat applyROI(const cv::Mat& frame) {
        cv::Mat mask = cv::Mat::zeros(frame.size(), frame.type());

        // Polygon ROI trên frame gốc (trước BEV)
        std::vector<cv::Point> roi_corners;
        roi_corners.push_back(cv::Point(frame.cols * 0.1, frame.rows * 1.0));
        roi_corners.push_back(cv::Point(frame.cols * 0.1, frame.rows * 0.6));
        roi_corners.push_back(cv::Point(frame.cols * 0.9, frame.rows * 0.6));
        roi_corners.push_back(cv::Point(frame.cols * 0.9, frame.rows * 1.0));
        std::vector<std::vector<cv::Point>> pts{roi_corners};
        cv::fillPoly(mask, pts, cv::Scalar(255,255,255));

        cv::Mat roi;
        cv::bitwise_and(frame, mask, roi);
        cv::Mat overlay = frame.clone();

        cv::fillPoly(overlay, pts, cv::Scalar(0, 255, 0));  // tô màu xanh lá
        cv::addWeighted(overlay, 0.3, frame, 0.7, 0, frame);
        return roi;
    }

    cv::Mat applyIPM(const cv::Mat& frame) {
        // Chọn 4 điểm nguồn (IPM)
        float offsetY = -80.0f;   // dịch lên trên
        float offsetX = 60.0f;    // kéo vào giữa

        cv::Point2f tl(width * 0.20f + offsetX, height * 0.65f + offsetY);
        cv::Point2f bl(0.0f   + offsetX,        height      + offsetY);
        cv::Point2f tr(width * 0.80f - offsetX, height * 0.65f + offsetY);
        cv::Point2f br(width  - offsetX,        height      + offsetY);

        std::vector<cv::Point2f> src_points = { tl, bl, tr, br };

        for (int i = 0; i < src_points.size(); i++) {
            cv::circle(frame, src_points[i], 5, cv::Scalar(0, 255, 0), -1);
        }

        // Điểm đích (góc nhìn từ trên xuống)
        std::vector<cv::Point2f> dst_points = {
            cv::Point2f(0, 0),
            cv::Point2f(0, height),
            cv::Point2f(width, 0),
            cv::Point2f(width, height)
        };

        // Tính toán và áp dụng IPM
        cv::Mat M = cv::getPerspectiveTransform(src_points, dst_points);
        cv::Mat bird_eye_view;
        cv::warpPerspective(frame, bird_eye_view, M, cv::Size(width, height));

        return bird_eye_view;
    }

    void slidingWindow(const cv::Mat& mask, std::vector<cv::Point>& left_points, std::vector<cv::Point>& right_points, cv::Mat& outImg) {
        int nwindows = 15;   // số lượng cửa sổ chia theo chiều dọc
        int margin = 60;    // nửa chiều rộng cửa sổ
        int minpix = 90;    // số pixel tối thiểu để dịch cửa sổ

        int height = mask.rows;
        int width = mask.cols;

        // --- 1. TÍNH TOÁN VÀ KIỂM TRA HISTOGRAM ---
        cv::Mat hist;
        cv::reduce(mask(cv::Rect(0, height / 2, width, height / 2)), hist, 0, cv::REDUCE_SUM, CV_32S);

        int midpoint = hist.cols / 2;
        
        // Tìm vị trí (iterator) và giá trị đỉnh
        auto it_left_ptr  = std::max_element(hist.begin<int>(), hist.begin<int>() + midpoint);
        auto it_right_ptr = std::max_element(hist.begin<int>() + midpoint, hist.end<int>());

        int leftx_base  = it_left_ptr - hist.begin<int>();
        int rightx_base = it_right_ptr - hist.begin<int>();
        
        // Đặt ngưỡng cường độ tối thiểu cho Histogram (Ngăn chặn nhiễu)
        // Nếu tổng cường độ quá nhỏ (dưới 1/5 chiều cao ảnh), coi như không có làn đường
        int hist_threshold = mask.rows / 5; 

        // Đánh dấu base không hợp lệ nếu cường độ quá thấp
        bool left_base_ok = (*it_left_ptr >= hist_threshold);
        bool right_base_ok = (*it_right_ptr >= hist_threshold);

        // Nếu cả hai đều không hợp lệ, thoát
        if (!left_base_ok && !right_base_ok) return;

        // --- 2. KHỞI TẠO VỊ TRÍ HIỆN TẠI ---
        int window_height = height / nwindows;
        
        // Chỉ khởi tạo current từ base hợp lệ. Nếu không hợp lệ, giữ nguyên giá trị (sẽ không được dùng)
        int leftx_current = left_base_ok ? leftx_base : -1; 
        int rightx_current = right_base_ok ? rightx_base : -1;

        std::vector<cv::Point> nonzero;
        cv::findNonZero(mask, nonzero);

        // --- 3. VÒNG LẶP SLIDING WINDOW ---
        for (int window = 0; window < nwindows; window++) {
            int win_y_low = height - (window + 1) * window_height;
            int win_y_high = height - window * window_height;

            std::vector<cv::Point> good_left, good_right;
            
            // --- XỬ LÝ LÀN ĐƯỜNG TRÁI ---
            if (left_base_ok) {
                cv::Rect left_win(leftx_current - margin, win_y_low, margin * 2, window_height);
                
                // Tìm điểm trong cửa sổ
                for (auto &p : nonzero) {
                    if (left_win.contains(p)) good_left.push_back(p);
                }

                // Điều kiện MINPIX có tác dụng: Chỉ vẽ và dịch chuyển nếu có đủ điểm
                if ((int)good_left.size() > minpix) {
                    // 1. Vẽ hình chữ nhật
                    cv::rectangle(outImg, left_win, cv::Scalar(0, 255, 0), 2);
                    
                    // 2. Thu thập điểm
                    left_points.insert(left_points.end(), good_left.begin(), good_left.end());

                    // 3. Dịch chuyển cửa sổ
                    int sumx = 0;
                    for (auto &p : good_left) sumx += p.x;
                    leftx_current = sumx / (int)good_left.size();
                }
                // ELSE: Nếu không đủ minpix, leftx_current giữ nguyên (tiếp tục tìm ở vị trí cũ)
            }

            // --- XỬ LÝ LÀN ĐƯỜNG PHẢI ---
            if (right_base_ok) {
                cv::Rect right_win(rightx_current - margin, win_y_low, margin * 2, window_height);

                // Tìm điểm trong cửa sổ
                for (auto &p : nonzero) {
                    if (right_win.contains(p)) good_right.push_back(p);
                }

                // Điều kiện MINPIX có tác dụng: Chỉ vẽ và dịch chuyển nếu có đủ điểm
                if ((int)good_right.size() > minpix) {
                    // 1. Vẽ hình chữ nhật
                    cv::rectangle(outImg, right_win, cv::Scalar(0, 255, 0), 2);
                    
                    // 2. Thu thập điểm
                    right_points.insert(right_points.end(), good_right.begin(), good_right.end());

                    // 3. Dịch chuyển cửa sổ
                    int sumx = 0;
                    for (auto &p : good_right) sumx += p.x;
                    rightx_current = sumx / (int)good_right.size();
                }
                // ELSE: Nếu không đủ minpix, rightx_current giữ nguyên
            }
        }
    }

    cv::Mat processMask(const cv::Mat& bird_eye_view) {
        // Chuyển sang HSV và lọc vùng sáng
        cv::Mat hsv, mask;
        cv::cvtColor(bird_eye_view, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(0, 0, 200), cv::Scalar(180, 40, 255), mask);

        return mask;
    }

    std::vector<std::vector<cv::Point>> findContoursInMask(const cv::Mat& mask) {
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        return contours;
    }

    cv::Vec3f fitPoly(const std::vector<cv::Point>& points, cv::Mat& outImg, bool isLeft) {
        if (points.size() < 2) return cv::Vec3f(0,0,0); // cần ít nhất 2 điểm

        // Tách X, Y từ vector points
        std::vector<float> x_vals, y_vals;
        for (const auto& pt : points) {
            x_vals.push_back((float)pt.x);
            y_vals.push_back((float)pt.y);
        }

        cv::Mat Y(y_vals.size(), 1, CV_32F, y_vals.data()); // biến độc lập
        cv::Mat X(x_vals.size(), 1, CV_32F, x_vals.data()); // biến phụ thuộc

        cv::Vec3f coeff_out(0,0,0);

        if (points.size() >= 3) {
            // ---- Fit bậc 2 ----
            cv::Mat A2(Y.rows, 3, CV_32F);
            for (int i = 0; i < Y.rows; ++i) {
                float y = Y.at<float>(i, 0);
                A2.at<float>(i, 0) = y * y;
                A2.at<float>(i, 1) = y;
                A2.at<float>(i, 2) = 1.0f;
            }

            cv::Mat coeffs2;
            bool ok = cv::solve(A2, X, coeffs2, cv::DECOMP_SVD);
            if (ok) {
                coeff_out = cv::Vec3f(coeffs2.at<float>(0), coeffs2.at<float>(1), coeffs2.at<float>(2));
            }
        }

        // Nếu chưa fit bậc 2 được (hoặc gần thẳng) thì fallback bậc 1
        if (coeff_out == cv::Vec3f(0,0,0) || std::fabs(coeff_out[0]) < 1e-6) {
            cv::Mat A1(Y.rows, 2, CV_32F);
            for (int i = 0; i < Y.rows; ++i) {
                float y = Y.at<float>(i, 0);
                A1.at<float>(i, 0) = y;
                A1.at<float>(i, 1) = 1.0f;
            }
            cv::Mat coeffs1;
            bool ok = cv::solve(A1, X, coeffs1, cv::DECOMP_SVD);
            if (ok) {
                coeff_out = cv::Vec3f(0, coeffs1.at<float>(0), coeffs1.at<float>(1));
            }
        }

        // ---- Vẽ đường fitted ----
        cv::Scalar lineColor = isLeft ? cv::Scalar(0,255,0) : cv::Scalar(0,0,255);
        for (int y = 0; y < outImg.rows; ++y) {
            float x = coeff_out[0]*y*y + coeff_out[1]*y + coeff_out[2];
            int ix = (int)std::round(x);
            if (ix >= 0 && ix < outImg.cols) {
                cv::circle(outImg, cv::Point(ix, y), 2, lineColor, -1);
            }
        }

        return coeff_out; // (a,b,c)
    }

    std::vector<cv::Point> computeCenterline(cv::Vec3f left_coeffs,
                                            cv::Vec3f right_coeffs,
                                            bool left_ok,
                                            bool right_ok,
                                            bool change_lane,
                                            cv::Mat& outImg,
                                            const cv::Mat& mask)
    {
        std::vector<cv::Point> centerline;

        // ==========================
        // CASE 1: CHANGE LANE
        // ==========================
        if (change_lane) {
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            if (!contours.empty()) {
                // chọn contour lớn nhất
                int bestIdx = -1;
                double maxArea = 0;
                for (int i = 0; i < contours.size(); i++) {
                    double area = cv::contourArea(contours[i]);
                    if (area > maxArea) {
                        maxArea = area;
                        bestIdx = i;
                    }
                }

                if (bestIdx >= 0) {
                    auto& bestContour = contours[bestIdx];

                    // --- Fit y = a*x^2 + b*x + c ---
                    std::vector<float> x_vals, y_vals;
                    for (auto &pt : bestContour) {
                        x_vals.push_back((float)pt.x);
                        y_vals.push_back((float)pt.y);
                    }

                    if (x_vals.size() >= 3) {
                        cv::Mat A(x_vals.size(), 3, CV_32F);
                        cv::Mat Y(y_vals.size(), 1, CV_32F, y_vals.data());

                        for (int i = 0; i < x_vals.size(); i++) {
                            float x = x_vals[i];
                            A.at<float>(i, 0) = x * x;
                            A.at<float>(i, 1) = x;
                            A.at<float>(i, 2) = 1.0f;
                        }

                        cv::Mat coeffs;
                        if (cv::solve(A, Y, coeffs, cv::DECOMP_SVD)) {
                            float a = coeffs.at<float>(0);
                            float b = coeffs.at<float>(1);
                            float c = coeffs.at<float>(2);

                            float cm_per_px = 0.2f;
                            float laneWidth_px = 70.0f / cm_per_px;
                            float halfLane_px = laneWidth_px / 2.0f;

                            // vẽ centerline giả tưởng
                            for (int x = 0; x < outImg.cols; x += 10) {
                                float y_center = a*x*x + b*x + c + halfLane_px;
                                int iy = (int)std::round(y_center);
                                if (iy >= 0 && iy < outImg.rows) {
                                    centerline.emplace_back(x, iy);
                                }
                            }

                            // Vẽ contour lane ngoài
                            cv::drawContours(outImg, contours, bestIdx, cv::Scalar(255, 0, 0), 2);

                            // Vẽ centerline liên tục
                            for (size_t i = 1; i < centerline.size(); i++) {
                                cv::line(outImg, centerline[i-1], centerline[i], cv::Scalar(0,255,255), 2);
                            }
                        }
                    }
                }
            }

            return centerline;
        }

        // ==========================
        // CASE 2: BÌNH THƯỜNG
        // ==========================
        float cm_per_px = 0.2f;
        float laneWidth_px = 35.0f / cm_per_px; // ~175 px

        for (int y = 0; y < outImg.rows; y += 10) {
            float x_left = -1, x_right = -1;

            if (left_ok && right_ok) {
                x_left  = left_coeffs[0]*y*y + left_coeffs[1]*y + left_coeffs[2];
                x_right = right_coeffs[0]*y*y + right_coeffs[1]*y + right_coeffs[2];
            }
            else if (left_ok) {
                x_left  = left_coeffs[0]*y*y + left_coeffs[1]*y + left_coeffs[2];
                x_right = x_left + laneWidth_px;
            }
            else if (right_ok) {
                x_right = right_coeffs[0]*y*y + right_coeffs[1]*y + right_coeffs[2];
                x_left  = x_right - laneWidth_px;
            }
            else {
                continue; // không detect được lane nào
            }

            float x_center = 0.5f * (x_left + x_right);
            cv::Point pt((int)x_center, y);
            centerline.push_back(pt);

            cv::circle(outImg, pt, 2, cv::Scalar(255,255,0), -1);
        }

        return centerline;
    }

    float computeLaneSlope(const cv::Vec3f& coeffs, float y) {
        // coeffs = (a, b, c) -> x = a*y^2 + b*y + c
        float a = coeffs[0];
        float b = coeffs[1];
        float c = coeffs[2];

        if (std::fabs(a) > 1e-6) {
            return 2*a*y + b; // slope của bậc 2
        } else {
            return b;         // slope của bậc 1
        }
    }

};

int main() {
    std::string videoPath = "D:/Downloads/sample_vid.mp4"; // Đường dẫn video
    LaneDetector detector(videoPath);
    detector.processFrame();
    return 0;
}
