#include <iostream>

#include <Eigen/Dense>

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

// --------------------------------------------------------------------------
  std::cout << std::endl << std::endl;
// --------------------------------------------------------------------------

  Affine3d T2 = Affine3d::Identity();
  T2.translation() = (VectorXd(3) << 1, 0, 0).finished();

  Affine3d T3 = Affine3d::Identity();
  T3.translation() = (VectorXd(3) << 5, -5, 0).finished();

  std::cout << (T2.inverse()*T3).matrix() << std::endl;

// --------------------------------------------------------------------------
  std::cout << std::endl << std::endl;
// --------------------------------------------------------------------------

  // 3x4 matrix
  std::cout << T3.affine().matrix() << std::endl;

  return 0;
}
