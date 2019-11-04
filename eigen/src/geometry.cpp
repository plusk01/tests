#include <iostream>

#include <Eigen/Dense>

#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>


using namespace std;
using namespace Eigen;

int main() {

  Affine3d T = Affine3d::Identity();
  std::cout << T.matrix() << std::endl;

// --------------------------------------------------------------------------
  std::cout << std::endl << std::endl;
// --------------------------------------------------------------------------

  Affine3d T1 = Affine3d::Identity();
  // T1.rotation() = AngleAxisd(1.59, Vector3d::UnitX());
  T1 = AngleAxisd(M_PI/2, Vector3d::UnitX());
  // T1.translation() = (VectorXd(3) << 0.7, 1.0, 56.2).finished();
  std::cout << T1.matrix() << std::endl;

  std::cout << std::endl;
  std::cout << T1.rotation() << std::endl;

  Quaterniond q(T1.rotation());
  std::cout << "[x y z w]" << std::endl;
  std::cout << q.coeffs() << std::endl;
  std::cout << std::endl;
  std::cout << q.coeffs().x() << std::endl;
  std::cout << q.coeffs().y() << std::endl;
  std::cout << q.coeffs().z() << std::endl;
  std::cout << q.coeffs().w() << std::endl;

  Vector3d x = (Vector3d() << 1, 2, 3).finished();
  Vector3d y = T1*x;
  std::cout << "y^T: " << y.transpose() << std::endl;

// --------------------------------------------------------------------------
  std::cout << std::endl << std::endl;
// --------------------------------------------------------------------------

  Affine3d T2 = Affine3d::Identity();
  T2.translation() = (VectorXd(3) << 1, 0, 0).finished();

  Affine3d T3 = Affine3d::Identity();
  T3.translation() = (VectorXd(3) << 5, -5, 0).finished();

  auto T4 = (T2.inverse()*T3);
  std::cout << T4.matrix() << std::endl;

  std::cout << T4.translation().x() << std::endl;

// --------------------------------------------------------------------------
  std::cout << std::endl << std::endl;
// --------------------------------------------------------------------------

  // 3x4 matrix
  std::cout << T3.affine().matrix() << std::endl;

// --------------------------------------------------------------------------
  std::cout << std::endl << std::endl;
// --------------------------------------------------------------------------

  cv::Mat matT3;
  Eigen::Matrix<double, 3, 4> TT = T3.affine();
  cv::eigen2cv(TT, matT3);
  std::cout << matT3 << std::endl;

// --------------------------------------------------------------------------
  std::cout << std::endl << std::endl;
// --------------------------------------------------------------------------

  cv::Mat m = cv::Mat::eye(3, 4, CV_64FC1);
  Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> result(m.ptr<double>(), m.rows, m.cols);

  result = (T2.inverse()*T3).affine().matrix();

  std::cout << m << std::endl;

// --------------------------------------------------------------------------
  std::cout << std::endl << std::endl;
// --------------------------------------------------------------------------

  {
    Matrix3d R;
    R << -0.4096,    0.5636,    0.7173,
          0.7846,   -0.1835,    0.5922,
          0.4654,    0.8054,   -0.3671;

    {
      Quaterniond q(R);
      std::cout << "w " << q.coeffs().w() << std::endl;
      std::cout << "x " << q.coeffs().x() << std::endl;
      std::cout << "y " << q.coeffs().y() << std::endl;
      std::cout << "z " << q.coeffs().z() << std::endl;
    }

    // now, extract ZYX intrinsic
    Vector3d eul = R.eulerAngles(2, 1, 0);
    std::cout << "eul (ZYX intrinsic): " << eul.transpose() << std::endl;
    Quaterniond q = AngleAxisd(eul[0], Vector3d::UnitZ())
                  * AngleAxisd(eul[1], Vector3d::UnitY())
                  * AngleAxisd(eul[2], Vector3d::UnitX());
    std::cout << std::endl;
    std::cout << q.toRotationMatrix() << std::endl << std::endl;

    // make a yaw-only quat
    Quaterniond qyaw(AngleAxisd(eul[0], Vector3d::UnitZ()));
    std::cout << std::endl;
    std::cout << qyaw.toRotationMatrix() << std::endl << std::endl;

  }

  return 0;
}
// 2.0519   -0.4841    1.9984