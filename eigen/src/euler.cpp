#include <iostream>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

// same as Matrix3x3(q).getEulerYPR or Matrix3x3(q).getRPY or tf.euler_from_quaternion(q, axes='sxyz')
// http://docs.ros.org/noetic/api/tf2/html/Matrix3x3_8h_source.html#l00294
// See Section 2.6 of https://www.geometrictools.com/Documentation/EulerAngles.pdf
// and https://www.gregslabaugh.net/publications/euler.pdf <-- explains why div by cos
Eigen::Vector3d sxyz_from_quaternion(const Eigen::Quaterniond& q)
{
  const Eigen::Matrix3d R = q.toRotationMatrix();

  // there are two solutions, not sure why...
  Eigen::Vector3d sxyz, sxyz2;

  // Check if pitch is at a singularity
  if (std::abs(R.row(2).x()) >= 1) {
    sxyz.z() = sxyz2.z() = 0;

    // From difference of angles formula
    const double delta = std::atan2(R.row(2).y(), R.row(2).z());
    if (R.row(2).x() < 0) { // gimbal locked down
      sxyz.y() = sxyz2.y() = M_PI_2;
      sxyz.x() = sxyz2.x() = delta;
    } else { // gimbal locked up
      sxyz.y() = sxyz2.y() = -M_PI_2;
      sxyz.x() = sxyz2.x() = delta;
    }
  } else {
    sxyz.y() = -std::asin(R.row(2).x());
    sxyz2.y() = M_PI - sxyz.y();

    const double cth  = std::cos(sxyz.y());
    const double cth2 = std::cos(sxyz2.y());

    sxyz.x()  = std::atan2(R.row(2).y() / cth, R.row(2).z() / cth);
    sxyz2.x() = std::atan2(R.row(2).y() / cth2, R.row(2).z() / cth2);

    sxyz.z()  = std::atan2(R.row(1).x() / cth, R.row(0).x() / cth);
    sxyz2.z() = std::atan2(R.row(1).x() / cth2, R.row(0).x() / cth2);
  }

  static constexpr int solution_number = 1;
  return ((solution_number == 1) ? sxyz : sxyz2);
}

int main(int argc, char const *argv[])
{
  // intrinsic 3-2-1 angles <--> extrinsic 1-2-3 (same values can be interpreted differently)
  double R = 0.2, P = 1, Y = 1; // written down as intrinsic 3-2-1
  Eigen::Quaterniond q1 = Eigen::AngleAxisd(Y, Eigen::Vector3d::UnitZ()) * // applied/interpreted as extrinsic 1-2-3
                          Eigen::AngleAxisd(P, Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(R, Eigen::Vector3d::UnitX());

  std::cout << R << " " << P << " " << Y << std::endl;
  std::cout << q1.w() << " " << q1.vec().transpose() << std::endl;
  std::cout << "-----------------------------------" << std::endl;
  auto sxyz = sxyz_from_quaternion(q1);
  std::cout << sxyz.x() << " " << sxyz.y() << " " << sxyz.z() << std::endl;

  Eigen::Quaterniond q2 = Eigen::AngleAxisd(sxyz.z(), Eigen::Vector3d::UnitZ()) * 
                          Eigen::AngleAxisd(sxyz.y(), Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(sxyz.x(), Eigen::Vector3d::UnitX());
  std::cout << q2.w() << " " << q2.vec().transpose() << std::endl;

  /**
   * Output from transformations.py
   * 
   * In [1]: q = [-0.151814, 0.460637, 0.37663, 0.78925] # x,y,z,w
   * 
   * In [2]: euler_from_quaternion(q, axes='sxyz')
   * Out[2]: (0.1999993126641404, 1.000000412724532, 0.9999996550375637)
   * 
   */ 

  return 0;
}