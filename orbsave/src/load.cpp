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

void loadYAML(std::vector<std::vector<cv::Mat>>& allDescriptors,
              std::vector<std::vector<cv::Point2f>>& allPoints)
{
  allDescriptors.clear();
  allPoints.clear();

  cv::FileStorage fs("images.yaml", cv::FileStorage::READ);

  int nrImages = 0;
  nrImages = fs["nrImages"];

  for (int i=0; i<nrImages; ++i) {
    allPoints.push_back({});
    allDescriptors.push_back({});

    cv::FileNode fn = fs["image" + std::to_string(i)];
    for (const auto& f : fn) {
      // feature point
      cv::Point2f pt{f["x"], f["y"]};
      allPoints.back().push_back(pt);

      // descriptor
      std::vector<uchar> dvec;
      f["d"] >> dvec;
      cv::Mat d(1, dvec.size(), CV_8U, dvec.data());
      allDescriptors.back().push_back(d.clone());
    }
  }
  fs.release();
}


// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------

int main(int argc, char const *argv[])
{
  std::vector<std::vector<cv::Mat>> allDescriptors;
  std::vector<std::vector<cv::Point2f>> allPoints;

  loadYAML(allDescriptors, allPoints);

  //
  // Print a few descriptors
  //

  std::cout << allDescriptors.front().front() << std::endl;

  //
  // Visualize images with keypoints
  //

  assert(allDescriptors.size() == NIMAGES);
  assert(allPoints.size() == NIMAGES);

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
