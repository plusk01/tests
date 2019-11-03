#include <iostream>

#include <Eigen/Dense>


using namespace std;
using namespace Eigen;

int main() {

  using MyMat = Eigen::Matrix<double, Eigen::Dynamic, 3>;
  MyMat M(4,3);

  M << 0.8, 0.9, 1.2,
       0.0, 0.1, 0.4,
       1.0, 2.0, 3.0,
       0.7, 0.7, 9.0;

  std::cout << M << std::endl;


  // --------------------------------------------------------------------------
  // --------------------------------------------------------------------------

  // This copies M to raw buffer
  double ptr[M.rows()*M.cols()];
  Eigen::Map<MyMat>(ptr, M.rows(), M.cols()) = M;

  std::cout << "mapped" << std::endl;

  // eigen is col-major order by default
  for (size_t i=0; i<M.rows(); ++i) {
    for (size_t j=0; j<M.cols(); ++j) { // note this is the slow way, for printing
      std::cout << ptr[i + M.rows()*j] << "   ";
    }
    std::cout << std::endl;
  }

  // --------------------------------------------------------------------------
  // --------------------------------------------------------------------------

  M.data()[0 + M.rows()*2] = 1.5;
  std::cout << M << std::endl;

  return 0;
}
