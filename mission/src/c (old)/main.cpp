#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <libcamera/libcamera.h>

int main() {
    // Define Gstreamer pipeline
    std::string pipeline = 
		"libcamerasrc ! "
		"video/x-raw,format=NV12,width=2304,height=1296,framerate=30/1 ! "
		"videoconvert,format=BGR ! "
		"appsink drop=true sync=false";

    // Open camera
    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    if (!cap.isOpened()) {
        std::cerr << "Error: could not open camera" << std::endl;
        return -1;
    }

    // Set capture resolution and FPS (adjust to your camera capabilities)
    int width  = 2304;
    int height = 1296;
    double fps = 30.0;
    
    std::cout << "Capture started: " << width << "x" << height << " @ " << fps << " FPS\n";

    // Output file and codec
    std::string outputFile = "raw.avi";
    // Try H264; fallback to MP4V if H264 not available
    int fourcc = cv::VideoWriter::fourcc('M','J','P','G');
    cv::VideoWriter writer;
    writer.open(outputFile, fourcc, fps, cv::Size(width, height), true);
    
    if (!writer.isOpened()) {
        std::cerr << "Warning: H264 writer failed, trying mp4v\n";
        fourcc = cv::VideoWriter::fourcc('m','p','4','v');
        writer.open(outputFile, fourcc, fps, cv::Size(width, height), true);
        
        if (!writer.isOpened()) {
            std::cerr << "Error: could not open VideoWriter\n";
            return -1;
        }
    }

    std::cout << "Recording to " << outputFile << "\n";

    cv::namedWindow("PiCam", cv::WINDOW_NORMAL);

    auto start = std::chrono::steady_clock::now();
    const double maxSeconds = 60.0; // Default duration; program will stop after this unless you press 'q'.
    while (true) {
        cv::Mat frame;
        if (!cap.read(frame) || frame.empty()) {
            std::cerr << "Warning: empty frame grabbed\n";
            continue;
        }

        writer.write(frame);
        cv::imshow("PiCam", frame);

        // Stop if 'q' pressed
        int key = cv::waitKey(1);
        if ((key & 0xFF) == 'q') break;

        // Stop after maxSeconds
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - start).count();
        if (elapsed >= maxSeconds) break;
    }

    cap.release();
    writer.release();
    cv::destroyAllWindows();
    std::cout << "Finished recording.\n";
    return 0;
}
