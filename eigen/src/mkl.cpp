#include <iostream>

#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

int main() {

  MatrixXd A = MatrixXd::Random(200,300);
  MatrixXd B = MatrixXd::Random(300,200);

  MatrixXd C = A*B;
  std::cout << C.size() << std::endl;

  return 0;
}