#include <iostream>

#include "opencv2/opencv.hpp"

cv::Mat K, frame;

// https://github.com/opencv/opencv/blob/master/samples/cpp/ffilldemo.cpp
void onMouse(int event, int x, int y, int d, void *ptr) {

    // Only continue if the left button was pressed
    if (event != cv::EVENT_LBUTTONDOWN) return;

    std::cout << "Clicked point: (" << x << ", " << y << ") pixels\t" << std::flush;

    std::vector<cv::Point2f> pt;
    pt.emplace_back(x, y);

    cv::undistortPoints(pt, pt, K, cv::noArray(), cv::noArray());
    std::cout << "(" << pt[0].x << ", " << pt[0].y << ") normalized image coordinates" << std::endl;

}

int main(int argc, char const *argv[]) {

    // Print OpenCV version
    std::cout << "OpenCV version: "
                << CV_MAJOR_VERSION << "." 
                << CV_MINOR_VERSION << "."
                << CV_SUBMINOR_VERSION
                << std::endl;
    
    cv::VideoCapture cap(0); // open the default camera

    // Microsoft LifeCam Intrinsics
    K = (cv::Mat_<double>(3, 3) << 590.0613663340589, 0, 320, 0, 590.0613663340589, 240, 0, 0, 1);
    std::cout << "Camera Matrix:\n" << K << std::endl;

    // Make sure that a camera was opened
    if (!cap.isOpened()) return -1;

    // Create the window that will display the output
    cv::namedWindow("test", cv::WINDOW_AUTOSIZE);

    // Callback so we can click and process based on pixel location
    cv::setMouseCallback("test", onMouse, NULL);

    while(1) {
        cap >> frame; // get a new frame from the camera

        cv::line(frame, cv::Point2f(0, 240), cv::Point2f(640, 240), cv::Scalar(0, 0, 255), 2);
        cv::line(frame, cv::Point2f(320, 0), cv::Point2f(320, 480), cv::Scalar(0, 0, 255), 2);

        cv::imshow("test", frame);

        if ((char)cv::waitKey(30) == (char)27) break;
    }

    // The camera will be cleaned up in the VideoCapture destructor.
    return 0;
}