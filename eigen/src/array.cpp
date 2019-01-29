#include <iostream>

#include <Eigen/Dense>

using namespace Eigen;

int main()
{

  Matrix3d A = (Matrix3d() << 1,-3,3, 3,5,3, 6,-6,4).finished();

  std::cout << "diag(A): " << A.diagonal().array().log().sum() << std::endl;

  return 0;
}