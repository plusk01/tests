#include <iostream>

#include <Eigen/Dense>

using namespace Eigen;

void dontWork(Vector2d& a)
{
  a.coeffRef(0) = 9;
  a.coeffRef(1) = .605;
}


void works(Ref<Vector2d> a)
{
  a.coeffRef(0) = 9;
  a.coeffRef(1) = .605;
}


int main()
{

  Matrix<double, 3, 10> A;
  A.setZero();

  works(A.col(0).head<2>());
  // dontWork(A.col(0).head<2>());

  std::cout << "A:" << std::endl << A << std::endl;

  return 0;
}