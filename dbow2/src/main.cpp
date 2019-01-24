#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>

#include "DBoW2/DBoW2.h"
#include "DBoW2/FORB.h"

/**
 *  https://stackoverflow.com/a/39780825/2392520
 *  cv::Mat::type
 *  +--------+----+----+----+----+------+------+------+------+
 *  |        | C1 | C2 | C3 | C4 | C(5) | C(6) | C(7) | C(8) |
 *  +--------+----+----+----+----+------+------+------+------+
 *  | CV_8U  |  0 |  8 | 16 | 24 |   32 |   40 |   48 |   56 |
 *  | CV_8S  |  1 |  9 | 17 | 25 |   33 |   41 |   49 |   57 |
 *  | CV_16U |  2 | 10 | 18 | 26 |   34 |   42 |   50 |   58 |
 *  | CV_16S |  3 | 11 | 19 | 27 |   35 |   43 |   51 |   59 |
 *  | CV_32S |  4 | 12 | 20 | 28 |   36 |   44 |   52 |   60 |
 *  | CV_32F |  5 | 13 | 21 | 29 |   37 |   45 |   53 |   61 |
 *  | CV_64F |  6 | 14 | 22 | 30 |   38 |   46 |   54 |   62 |
 *  +--------+----+----+----+----+------+------+------+------+
 */

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void loadFeatures(std::vector<std::vector<cv::Mat>>& features);
void changeStructure(const cv::Mat& plain, std::vector<cv::Mat>& out);

// number of training images
const int NIMAGES = 4;

const char IMG_ROOT[] = "../images/";

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

int main(int argc, char const *argv[])
{
  std::vector<std::vector<cv::Mat>> features;
  loadFeatures(features);


  std::cout << "Images: " << features.size() << std::endl;
  for (size_t i=0; i<features.size(); ++i) {
    std::cout << "  Image " << (i+1) << ": " << features[i].size() << std::endl;

    std::cout << "    Feature 0: " << std::endl;
    std::cout << "      - rows: " << features[i].front().rows << std::endl;
    std::cout << "      - cols: " << features[i].front().cols << std::endl;
    std::cout << "      - type: " << features[i].front().type() << std::endl;
    std::cout << "      - raw: " << features[i].front() << std::endl;
    std::cout << "      - str: " << DBoW2::FORB::toString(features[i].front()) << std::endl;

    std::cout << std::endl;
  }

  return 0;
}

// ----------------------------------------------------------------------------

void loadFeatures(std::vector<std::vector<cv::Mat>>& features)
{
  features.clear();
  features.reserve(NIMAGES);

  cv::Ptr<cv::ORB> orb = cv::ORB::create();

  std::cout << "Extracting ORB features..." << std::endl;
  for (int i=0; i<NIMAGES; ++i) {
    std::stringstream ss;
    ss << IMG_ROOT << "image" << i << ".png";

    cv::Mat image = cv::imread(ss.str(), 0);
    cv::Mat mask;
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;

    // Defaults:
    // 500 keypoints max
    // Lb = 256 bits (32 bytes) (desc length);
    // Sb = 31 (patch size)
    // 8 pyramidal levels with scale factor of 1.2
    orb->detectAndCompute(image, mask, keypoints, descriptors);

    features.push_back(std::vector<cv::Mat>());
    changeStructure(descriptors, features.back());
  }
}

// ----------------------------------------------------------------------------

void changeStructure(const cv::Mat& plain, std::vector<cv::Mat>& out)
{
  out.resize(plain.rows);

  for (int i=0; i<plain.rows; ++i) {
    out[i] = plain.row(i);
  }
}
