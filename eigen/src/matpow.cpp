#include <iostream>

#include <unsupported/Eigen/MatrixFunctions>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

int main() {

	Matrix3d A;
	A << 1, 2, 3,
	     4, 5, 6,
	     7, 8, 9;

	cout << "The matrix A:" << endl;
	cout << A << endl;

	cout << "The matrix A^0:" << endl;
	cout << A.pow(0) << endl;

	cout << "The matrix A^1:" << endl;
	cout << A.pow(1) << endl;

	cout << "The matrix A^2:" << endl;
	cout << A.pow(2) << endl;

	return 0;
}