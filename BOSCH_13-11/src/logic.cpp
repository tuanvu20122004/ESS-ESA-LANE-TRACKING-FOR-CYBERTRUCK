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
      udp_send("192.168.1.102", 9996),
      logger("Curvature.txt"),
      //udp_send1("192.168.1.103",9997),
      logger1("steering.txt"),
      logger2("Yaw.txt") {
    // Khởi tạo MPC
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

            // if(!detector.getBirdEyeView().empty()){              
            //      cv::imshow("Bird_eye_view", detector.getBirdEyeView());
	        // }    
            /*if(!detector.getMask().empty()){
                cv::imshow("Mask",detector.getMask());
            }*/
            // if(!detector.getFrameResize().empty()){              
            //      cv::imshow("Resize", detector.getFrameResize());     
            // }
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

            detector.processFrame(frame_local);
            std::vector<cv::Point> centerline = detector.getCenterline(); 
            cv::Mat birdEyeView = detector.getBirdEyeView();        
            MpcState state = mpc.computeMpcParameters(centerline, birdEyeView); 
            // Gui anh ve server
            cv::Mat bev = detector.getBirdEyeView(); // Bird eye view perspective
            //cv::Mat bev1 = detector.getFrameResize();  // Raw frame after resize

            if(!bev.empty()){
                //GUI Bird_eye_view 
                udp_send.sendFrame(bev,60);
                std::this_thread::sleep_for(std::chrono::milliseconds(40));
            }
            /*if(!bev1.empty()){
                //GUI Frame_size
                udp_send1.sendFrame(bev1,60);
                std::this_thread::sleep_for(std::chrono::milliseconds(33));
            }*/
            auto now = std::chrono::steady_clock::now();
            double proc_time = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_send).count();
            std::cout << "[MPC] Processing time: " << proc_time << " ms" << std::endl;
            if(std::chrono::duration_cast<std::chrono::milliseconds>(now-last_send).count() >= 50){
                last_send = now;
                if (state.is_valid) {
		//std::cout << "bien state hoat dong" << std::endl;
                float steering = mpc.computeSteeringAngle(state, desired_velocity);
                
                // if(steering<0.0f){
                //     steering = steering+3.0f;
                // }

		        std::cout << "Gia tri goc lai" << steering << std::endl;
                logger1.log("Steering",steering);
                //logger1.log("Lateral_Dev",state.lateral_deviation);
                //logger2.log("Yaw_Angle",state.yaw_angle);
                //logger.log("Curvature",state.curvature[0]);
                int servo = static_cast<int>(std::lround(97.0f + steering)); 
                //logger.log("Send_for_Stm32",servo);
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
