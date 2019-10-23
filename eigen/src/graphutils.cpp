#include <iostream>

#include <Eigen/Dense>

#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>


using namespace std;
using namespace Eigen;


static Eigen::MatrixXd pdistmat(const Eigen::MatrixXd& M)
{
  int n = M.rows();

  Eigen::VectorXd N = M.rowwise().squaredNorm();
  Eigen::MatrixXd D = N.replicate(1, n) + N.transpose().replicate(n, 1);
  D.noalias() -= 2. * M * M.transpose();
  D = D.array().sqrt(); 

  return D;
}


int main() {

  Eigen::Matrix<double, Eigen::Dynamic, 3> M(4,3);

  M << 0.8, 0.9, 1.2,
       0.0, 0.1, 0.4,
       1.0, 2.0, 3.0,
       0.7, 0.7, 9.0;

  std::cout << M << std::endl;

  std::cout << pdistmat(M) << std::endl;


  // --------------------------------------------------------------------------
  // --------------------------------------------------------------------------


  std::vector<uint> I = {0, 2, 3, 1};
  Eigen::PermutationMatrix<Dynamic, Dynamic, uint> perm(Eigen::Map<Eigen::Matrix<uint,Dynamic,1>>(I.data(), I.size()));
  std::cout << "Permutation: " << std::endl;
  std::cout << perm.toDenseMatrix() << std::endl;
  std::cout << "indices: " << perm.indices().transpose() << std::endl;

  std::cout << "Inverse Permutation: " << std::endl;
  Eigen::PermutationMatrix<Dynamic, Dynamic, uint> iperm = perm.transpose();
  std::cout << iperm.toDenseMatrix() << std::endl;
  std::cout << "indices: " << iperm.indices().transpose() << std::endl;

  return 0;
}
