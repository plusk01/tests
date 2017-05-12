#include <iostream>

#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

int main() {

	VectorXd v(6);
	v << 1,
		 2,
		 3,
	     4,
	     5,
	     6;

	cout << "The vector v:" << endl;
	cout << v << endl;

	MatrixXd A = Map<Matrix<double, 3, 2, RowMajor>>(v.data());

	cout << "The (3, 2) matrix A:" << endl;
	cout << A << endl;

	return 0;
}