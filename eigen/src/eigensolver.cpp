#include <iostream>

#include <Eigen/Dense>

using namespace Eigen;

int main() {

	Matrix3d A = (Matrix3d() << 1,-3,3, 3,-5,3, 6,-6,4).finished();

	std::cout << "Find the eigen values of the matrix, A:" << std::endl;
	std::cout << A << std::endl;

	EigenSolver<Matrix3d> es(A);
	std::cout << "The eigenvalues of A are:" << std::endl << es.eigenvalues() << std::endl;
	std::cout << "The matrix of eigenvectors, V, is:" << std::endl << es.eigenvectors() << std::endl;

	std::cout << std::endl << "The real parts:" << std::endl;
	std::cout << "The eigenvalues of A are:" << std::endl << es.eigenvalues().real() << std::endl;
	std::cout << "The matrix of eigenvectors, V, is:" << std::endl << es.eigenvectors().real() << std::endl;

	return 0;
}