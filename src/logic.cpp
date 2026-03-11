#include "logic.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <pthread.h>

void bindToCore(int core_id) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core_id, &cpuset);

    // Bind thread to the specified core
    pthread_t current_thread = pthread_self();
    int result = pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset);
    if (result != 0) {
        std::cerr << "[LOGIC] Error setting thread affinity." << std::endl;
    } else {
        std::cout << "[LOGIC] Thread bound to core " << core_id << "." << std::endl;
    }
}

Logic::Logic(const std::string& videoPath)
    : detector(videoPath, 640, 480), 
      comm("/dev/ttyACM0", 115200),  
      udp_send("192.168.1.100", 9996),
      //logger("Curvature.txt"),
      //udp_send1("192.168.1.103",9997),
    //   logger1("steering.txt"),
    //   logger2("Yaw.txt") {
    // Khởi tạo MPC
      distance_detector("/home/pi/Documents/yolov8n.onnx", 850.0f)
{
    mpc.init(1000.0f, 50.0f, 5.0f); 
    mpc.debugMatrices(); //x
    mpc.setVehicleParams(0.2515f, 2.3f, 0.132f, 0.12f, 0.04f, 0.02f, 0.04f);

    if (!detector.isOpened()) {
        std::cerr << "[LOGIC] LaneDetector/Camera không mở được." << std::endl;
        throw std::runtime_error("LaneDetector not opened");
    }
    std::cout << "[LOGIC] MPC initialized." << std::endl;
}

void Logic::run() { 
    // ---- Camera thread ----
    std::thread camera_thread([&]() {
        bindToCore(0); // Bind camera thread to core 0
        cv::Mat frame;
        //cv::namedWindow("Live Feed", cv::WINDOW_AUTOSIZE);  

        while (running.load()) {
            if (!detector.getFrame(frame)) {  
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            {
                std::lock_guard<std::mutex> lock(frame_mutex); 
                latest_frame = frame; 
            }

            int key = cv::waitKey(1);  
            if (key == 27 || key == 'q' || key == 'Q') {
                running.store(false);
                break;
            }
        }
    });

    // ---- MPC thread ----
    std::thread mpc_thread([&]() {
        bindToCore(0);
        cv::Mat frame_local;
        auto last_send = std::chrono::steady_clock::now();
        static float prev_steering = 0;
        while (running.load()) {
            auto start_time = std::chrono::steady_clock::now();
            {
                std::lock_guard<std::mutex> lock(frame_mutex);
                if (!latest_frame.empty()) {
                    frame_local = latest_frame;
                } else {
                    frame_local.release();
                }
            }

            if (frame_local.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10)); 
                continue;
            }

            // Tách 2 nhánh:
            // 1) frame_display: để detect vật thể + vẽ bbox + gửi UDP
            // 2) frame_lane: để xử lý lane, không bị nhiễu bởi bbox/text
            cv::Mat frame_display = frame_local.clone();
            cv::Mat frame_lane    = frame_local.clone();
            // Detect vật thể và vẽ khoảng cách lên frame gốc
            
            //distance_detector.detectAndDraw(frame_display);

            // Lane detection dùng frame sạch
            detector.processFrame(frame_lane);

            std::vector<cv::Point> centerline = detector.getCenterline();
            cv::Mat birdEyeView = detector.getBirdEyeView();

            MpcState state = mpc.computeMpcParameters(centerline, birdEyeView);

            // Gửi ảnh detect vật thể về server
            if (!frame_display.empty()) {
                udp_send.sendFrame(birdEyeView, 80);
                std::this_thread::sleep_for(std::chrono::milliseconds(40));
            }

            if (udp.receiveDistance()) {
            std::cout << "Latest distance: " << udp.getLatestDistance() << " m\n";
            std::cout << "Average distance: " << udp.getAverageDistance() << " m\n";
    }
            auto now = std::chrono::steady_clock::now();
            double proc_time = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_send).count();

            if(std::chrono::duration_cast<std::chrono::milliseconds>(now-last_send).count() >= 130)
            {
                last_send = now;
                if (state.is_valid) 
                {

                    float steering = mpc.computeSteeringAngle(state, desired_velocity);

                    //std::cout << "Gia tri goc lai mpc: " << steering << std::endl;
                    
                    steering = 0.010f*pow(steering,3) + 1.5f*steering;
                    
                    
                   // logger1.log("Steering",steering);

                    if (steering <= -25)
                        steering = -25;
                    else if (steering >= 25)
                        steering = 25;
                    

                    //std::cout << "Gia tri goc lai qua noi suy: " << steering << std::endl;

                    int servo = static_cast<int>(std::lround(97.0f + steering)); 

                    comm.sendCommands(desired_velocity,servo);
                }
            }
        }
    });

    // Chờ cho các luồng kết thúc
    while (running.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));  
    }

    if (camera_thread.joinable()) camera_thread.join();
    if (mpc_thread.joinable()) mpc_thread.join();

    //cv::destroyAllWindows();

    std::cout << "[LOGIC] Stopped cleanly." << std::endl;
}
