#include <iostream>
#include <sstream>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

// image information
const char IMG_ROOT[] = "../images/";
const int NIMAGES = 2;

// ----------------------------------------------------------------------------

void changeStructure(const cv::Mat& plain, std::vector<cv::Mat>& out)
{
  out.resize(plain.rows);

  for (int i=0; i<plain.rows; ++i) {
    out[i] = plain.row(i);
  }
}

// ----------------------------------------------------------------------------

void extract(std::vector<std::vector<cv::Mat>>& allDescriptors,
             std::vector<std::vector<cv::Point2f>>& allPoints)
{
  allDescriptors.clear();
  allDescriptors.reserve(NIMAGES);

  allPoints.clear();
  allPoints.reserve(NIMAGES);

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

    std::vector<cv::Point2f> pts;
    for (const auto& kp : keypoints) {
      pts.push_back(kp.pt);
    }
    allPoints.push_back(pts);

    allDescriptors.push_back(std::vector<cv::Mat>());
    changeStructure(descriptors, allDescriptors.back());
  }
}

// ----------------------------------------------------------------------------

void saveYAML(const std::vector<std::vector<cv::Mat>>& allDescriptors,
              const std::vector<std::vector<cv::Point2f>>& allPoints)
{
  cv::FileStorage fs("images.yaml", cv::FileStorage::WRITE);

  fs << "nrImages" << static_cast<int>(allDescriptors.size());

  for (size_t i=0; i<allDescriptors.size(); ++i) {
    fs << "image" + std::to_string(i) << "[";

    const size_t nrFeatures = allDescriptors[i].size();
    std::cout << "Image " << i << ": " << nrFeatures << " features" << std::endl;
    for (size_t j=0; j<nrFeatures; ++j) {
      fs << "{:";
      fs << "x" << allPoints[i][j].x;
      fs << "y" << allPoints[i][j].y;
      fs << "d" << "[:";
      for (size_t k=0; k<allDescriptors[i][j].cols; ++k) {
        fs << allDescriptors[i][j].at<uchar>(0, k);
      }
      fs << "]";
      fs << "}";
    }
    fs << "]";
  }
  fs.release();
}


// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------

int main(int argc, char const *argv[])
{
  std::vector<std::vector<cv::Mat>> allDescriptors;
  std::vector<std::vector<cv::Point2f>> allPoints;
  extract(allDescriptors, allPoints);

  saveYAML(allDescriptors, allPoints);

  //
  // Visualize images with keypoints
  //

  for (size_t i=0; i<NIMAGES; ++i) {
    std::stringstream ss;
    ss << IMG_ROOT << "image" << i << ".png";
    cv::Mat img = cv::imread(ss.str(), 1);

    const auto& points = allPoints[i];

    for (const auto& pt : points) {
      cv::circle(img, pt, 2, cv::Scalar(0,0,255), -1);
    }

    cv::imshow("Image " + std::to_string(i), img);
  }

  // break on escape
  while (1) {
    if (cv::waitKey(0) == (char)27) {
      break;
    }
  }

  return 0;
}
