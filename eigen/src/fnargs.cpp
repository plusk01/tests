#include <iostream>

#include <Eigen/Dense>

/**
 * @brief      First-order lowpass filter following
 * 
 *                y = alpha*y + (1-alpha)*x
 *
 * @param[in]  alpha  LPF parameter in [0 1]. Zero means no filtering.
 * @param[in]  meas   The new measurement, x.
 * @param      out    The current filtered value and output, y.
 *
 * @tparam     Derived  The templated type of the signal to LPF.
 */
template<typename T>
static void LPF(float alpha, const T& meas,
                T& out)
{
  //Eigen::MatrixBase<Derived>& _out = 
  //  const_cast<Eigen::MatrixBase<Derived>&>(out);
  out = alpha*out + (1-alpha)*meas;
}

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

  // =================================================

  Vector3f y = (Vector3f() << 2,2,2).finished();
  Vector3f x = (Vector3f() << 4,4,4).finished();

  std::cout << "y: " << y.transpose() << std::endl;
  std::cout << "x: " << x.transpose() << std::endl;
  LPF(0.6, x, y);
  std::cout << "filtered: " << y.transpose() << std::endl;

  return 0;
}
