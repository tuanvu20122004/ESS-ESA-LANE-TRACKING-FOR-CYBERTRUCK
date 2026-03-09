#include <iostream>
#include <stdexcept>
#include "logic.hpp"

int main(int argc, char* argv[]) {
    std::cout << "=== Lane Keeping System with MPC Controller ===\n";

    std::string videoPath;
    if (argc > 1) {
        videoPath = argv[1];
        std::cout << "[INFO] Using input source: " << videoPath << "\n";
    } else {
        videoPath = "/dev/video0"; // giữ nguyên mặc định của bạn
        std::cout << "[INFO] No input path provided. Using camera: " << videoPath << "\n";
        std::cout << "[HINT] Chạy: ./app /dev/video0  (USB cam)  hoặc đường dẫn file video\n";
    }

    try {
        Logic logic(videoPath);
        logic.run();
    } catch (const std::exception& e) {
        std::cerr << "[FATAL] " << e.what() << "\n";
        return 1;
    }

    std::cout << "Program finished.\n";
    return 0;
}
