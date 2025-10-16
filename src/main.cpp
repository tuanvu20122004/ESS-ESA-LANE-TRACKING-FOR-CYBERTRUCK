#include <iostream>
#include "logic.hpp"

int main(int argc, char* argv[]) {
    std::cout << "=== Lane Keeping System with MPC Controller ===" << std::endl;

    std::string videoPath;
    if (argc > 1) {
    videoPath = argv[1];
    std::cout << "[INFO] Using input source: " << videoPath << std::endl;
   } 
    else {
    videoPath = "/dev/video0"; 
    std::cout << "[INFO] No input path provided. Using camera: " << videoPath << std::endl;
   }

    Logic logic(videoPath);
    logic.run();

    std::cout << "Program finished." << std::endl;
    return 0;
}
