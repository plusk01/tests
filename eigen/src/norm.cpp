#include <iostream>

#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

int main() {

	MatrixXd A(3, 2);
	A << 1, 2,
		 3, 4,
	     5, 6;

     // The Frobenius matrix norm is the same as the 2-norm of
     // a vectorized matrix, i.e., in MATLAB
     // >> norm(A,'F') == norm(A(:),2)

	cout << "The matrix A, Frobenius norm:" << endl;
	cout << A << endl; 

	VectorXd norms = A.rowwise().norm();

	cout << "The (3, 2) matrix A:" << endl;
	cout << norms << endl;

	return 0;
}