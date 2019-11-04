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

  // convert to std vector
  Eigen::Matrix<uint, Dynamic, Dynamic> mat = perm.indices();
  // https://stackoverflow.com/a/26094708/2392520
  std::vector<uint> v; v.resize(mat.size());
  Eigen::Matrix<uint, Dynamic, 1>::Map(&v[0], v.size()) = mat;
  // or: https://stackoverflow.com/a/26094702/2392520
  // v = std::vector<uint>(mat.data(), mat.data() + mat.size());

  std::cout << "vector: ";
  for (size_t i=0; i<v.size(); ++i) std::cout << v[i] << " ";
  std::cout << std::endl;

  return 0;
}
