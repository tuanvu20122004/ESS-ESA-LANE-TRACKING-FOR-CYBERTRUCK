#include "logic.hpp"
#include <iostream>
#include <thread>
#include <chrono>

Logic::Logic(const std::string& videoPath)
    : detector(videoPath, 640, 480),
      comm("/dev/ttyACM0", 115200) {

    // Khởi tạo MPC
    mpc.init(1000.0f, 50.0f, 5.0f);
    mpc.setVehicleParams(0.2515f, 2.3f, 0.132f, 0.12f, 0.04f, 0.02f, 0.04f);

    if (!detector.isOpened()) {
        std::cerr << "[LOGIC] LaneDetector/Camera không mở được." << std::endl;
        throw std::runtime_error("LaneDetector not opened");
    }
    std::cout << "[LOGIC] MPC initialized." << std::endl;
}

void Logic::run() {
    Logger logger("performance_log.txt");

    // ---- Thread chụp camera: CHỈ LaneDetector được quyền đọc frame ----
    std::thread camera_thread([&]() {
        cv::Mat frame;
        cv::namedWindow("Live Feed", cv::WINDOW_AUTOSIZE);

        while (running.load()) {
            if (!detector.getFrame(frame)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }

            {
                std::lock_guard<std::mutex> lock(frame_mutex);
                latest_frame = frame.clone();
            }

            // Preview (có thể tắt nếu muốn tiết kiệm CPU)
            cv::imshow("Live Feed", frame);
            int key = cv::waitKey(1);
            if (key == 27 || key == 'q' || key == 'Q') { // ESC / q
                running.store(false);
                break;
            }
        }
    });

    // ---- Thread MPC: xử lý ảnh + tính toán điều khiển ----
    std::thread mpc_thread([&]() {
        cv::Mat frame_local;

        while (running.load()) {
            {
                std::lock_guard<std::mutex> lock(frame_mutex);
                if (!latest_frame.empty()) {
                    frame_local = latest_frame.clone();
                } else {
                    frame_local.release();
                }
            }

            if (frame_local.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }

            // Xử lý & lấy trạng thái MPC từ LaneDetector
            detector.processFrame(frame_local);
            MpcState state = detector.getMpcState();

            if (state.is_valid) {
                float steering = mpc.computeSteeringAngle(state, desired_velocity);
                int servo = 93 + static_cast<int>(steering);
                comm.sendCommands(desired_velocity, servo);
            }

            // (Tuỳ chọn) logger.log(...) nếu muốn
        }
    });

    // ---- Vòng chờ chính ----
    while (running.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // ---- Dọn dẹp ----
    if (camera_thread.joinable()) camera_thread.join();
    if (mpc_thread.joinable())   mpc_thread.join();
    cv::destroyAllWindows();

    std::cout << "[LOGIC] Stopped cleanly." << std::endl;
}
