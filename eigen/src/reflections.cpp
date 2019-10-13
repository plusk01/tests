#include <iostream>

#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

int main() {

  static const Eigen::PermutationMatrix<3> NED_ENU_REFLECTION_1(Eigen::Vector3i(1,0,2));
  static const Eigen::DiagonalMatrix<double,3> NED_ENU_REFLECTION_2(1,1,-1);

  std::cout << NED_ENU_REFLECTION_1.toDenseMatrix() << std::endl;


  return 0;
}
