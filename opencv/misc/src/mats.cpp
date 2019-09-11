#include <iostream>
#include <vector>

#include <opencv2/core.hpp>


int main(int argc, char** argv)
{

    // ------------------------------------------------------------------------
    // vector to mat
    // ------------------------------------------------------------------------

    std::vector<cv::Point2f> pts;
    pts.emplace_back(1.0,2.0);
    pts.emplace_back(0,0);
    pts.emplace_back(-7.0,6.5);

    bool copy = false;
    cv::Mat test(pts, copy);

    assert(test.type() == 13); // CV_32FC2
    std::cout << test.rows << "x" << test.cols << std::endl;
    std::cout << "dims: " << test.dims << std::endl;
    std::cout << "channels: " << test.channels() << std::endl;


    // ------------------------------------------------------------------------
    // reshape mat
    // ------------------------------------------------------------------------
    
    cv::Mat test2 = test.reshape(1);
    assert(test2.type() == 5); // CV_32FC1
    std::cout << test2.rows << "x" << test2.cols << std::endl;
    std::cout << "dims: " << test2.dims << std::endl;
    std::cout << "channels: " << test2.channels() << std::endl;

    return 0;        
}

