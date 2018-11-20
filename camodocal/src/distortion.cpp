#include <iostream>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <camodocal/camera_models/PinholeCamera.h>
#include <camodocal/camera_models/CameraFactory.h>


camodocal::CameraPtr m_camera;
double fx, fy;

void loadIntrinsics(const std::string& calib_file) {
  m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(calib_file);

  // get focal lengths
  auto ph_cam = boost::reinterpret_pointer_cast<camodocal::PinholeCamera>(m_camera);
  fx = ph_cam->getParameters().fx();
  fy = ph_cam->getParameters().fy();
}

// ----------------------------------------------------------------------------

cv::Mat showUndistored(const cv::Mat& distorted) {
  static const int ROWS = m_camera->imageHeight();
  static const int COLS = m_camera->imageWidth();
  static constexpr int offset = 600;

  cv::Mat undistorted(ROWS + offset, COLS + offset, CV_8UC1, cv::Scalar(0));

  std::vector<Eigen::Vector2d> pts_dist;    // these are pixels
  std::vector<Eigen::Vector2d> pts_undist;  // these are calibrated (bearing vectors)

  // Create calibrated bearing vectors (using K and undistorting) corresponding to each pixel position.
  for (int i=0; i<ROWS; ++i) {
    for (int j=0; j<COLS; ++j) {
      // calibrate pixel point to obtain bearing vector
      Eigen::Vector2d a(j, i);  // pixels
      Eigen::Vector3d b;        // bearing vector (calibrated)
      m_camera->liftProjective(a, b);
      // store for later
      pts_dist.push_back(a);
      pts_undist.push_back(Eigen::Vector2d(b.x()/b.z(), b.y()/b.z()));
      // std::cout << "(" << a.x() << ", " << a.y() << ") --> (" << b.x()/b.z() << ", " << b.y()/b.z() << ")" << std::endl;
    }
  }

  // Take each bearing vector and reproject (using K, but without distortion) to a pixel position.
  for (size_t i=0; i<pts_undist.size(); ++i) {
    int u, v;

    u = pts_undist[i].x()*fx + COLS/2.0;
    v = pts_undist[i].y()*fy + ROWS/2.0;

    // make sure we aren't accessing invalid memory
    bool inHeight = (v + offset/2.0 >= 0) && (v + offset/2.0 < ROWS + offset);
    bool inWidth  = (u + offset/2.0 >= 0) && (u + offset/2.0 < COLS + offset);
    if (inHeight && inWidth) {
      undistorted.at<uchar>(v + offset/2.0, u + offset/2.0) = distorted.at<uchar>(pts_dist[i].y(), pts_dist[i].x());
    } else {
      // std::cout << "ERR " << i << std::endl;
    }

  }


  return undistorted;
}

// ----------------------------------------------------------------------------

int main(int argc, char const *argv[]) {

  if (argc != 2) {
    std::cout << "Please supply config file location" << std::endl;
    return 1;
  }

  // Setup CamOdoCal camera model
  std::string config_file = argv[1];
  std::cout << "CONFIG: " << config_file << std::endl;
  loadIntrinsics(config_file);

  std::cout << "DIMS: " << m_camera->imageWidth() << " x " << m_camera->imageHeight() << std::endl;

  // open video camera
  cv::VideoCapture cap(0);
  if (!cap.isOpened()) return 2;

  // setup viewing window
  cv::namedWindow("view", cv::WINDOW_NORMAL);

  for (;;) {
    // retrieve next frame
    cv::Mat frame;
    cap >> frame;

    // convert to greyscale
    cv::Mat grey;
    cv::cvtColor(frame, grey, CV_BGR2GRAY);

    cv::Mat undistorted = showUndistored(grey);

    if (!undistorted.empty()) {
      cv::imshow("view", undistorted);
    }
    cv::waitKey(1);
  }

  return 0;
}