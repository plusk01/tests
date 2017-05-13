#include <iostream>

#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

int main() {

	MatrixXd A(3, 2);
	A << 1, 2,
		 3, 4,
	     5, 6;

	cout << "The matrix A:" << endl;
	cout << A << endl;

	VectorXd norms = A.rowwise().norm();

	cout << "The (3, 2) matrix A:" << endl;
	cout << norms << endl;

	return 0;
}