#include <iostream>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

using SpMat = Eigen::SparseMatrix<double>;
using SpVec = Eigen::SparseVector<double>;
using T = Eigen::Triplet<double>;

void vectorize(const SpMat& X, SpMat& x)
{
  x.resize(X.size(), 1);
  x.reserve(X.nonZeros());
  x.startVec(0);
  for (size_t j=0; j<X.cols(); ++j) {
    for (SpMat::InnerIterator it(X, j); it; ++it) {
      x.insertBack(j*X.rows() + it.row(), 0) = it.value();
    }
  }

}

int main() {

  std::vector<T> coeffs;
  coeffs.emplace_back(0,0,7);
  coeffs.emplace_back(2,0,-1);
  coeffs.emplace_back(0,1,8);
  coeffs.emplace_back(2,2,1);

  SpMat A(3,3);
  A.setFromTriplets(coeffs.begin(), coeffs.end());

  std::cout << A.transpose() << std::endl;

  SpMat a;
  vectorize(A, a);

  std::cout << A << std::endl;
  std::cout << "------------------" << std::endl;
  std::cout << a << std::endl;

  std::cout << a  << std::endl;

	return 0;
}
