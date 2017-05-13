#include <iostream>

#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

// Use conservativeResize(shape) if you don't care about what the values are
// initialized to.
// Use conservativeResizeLike() if you want to initialize the values
//
// http://stackoverflow.com/questions/25317687/conservativeresize-with-zero-values-for-the-new-values
// https://eigen.tuxfamily.org/dox/classEigen_1_1PlainObjectBase.html#a4ece7540eda6a1ae7d3730397ce72bec

int main() {

	MatrixXd A(3, 3);
	A << 1, 2, 3,
		 4, 5, 6,
		 7, 8, 9;

	cout << "The matrix A:" << endl;
	cout << A << endl;

	A.conservativeResize(4, 4);

	cout << "The upsized matrix A:" << endl;
	cout << A << endl << endl << endl;

	MatrixXd B(5, 5);
	B << 1, 2, 3, 4, 5,
		 6, 7, 8, 9, 0,
		 9, 8, 7, 6, 5,
		 4, 3, 2, 1, 0,
		 1, 2, 3, 4, 5;

	cout << "The matrix B:" << endl;
	cout << B << endl;

	B.conservativeResize(3, 3);

	cout << "The downsized matrix B:" << endl;
	cout << B << endl << endl << endl;

	// ==========================================

	VectorXd v(4);
	v << 1, 2, 3, 4;

	cout << "The vector v: " << v.transpose() << endl;

	v.conservativeResizeLike(Eigen::VectorXd::Zero(10));

	// v.block(4, 0, 6, 1) = VectorXd::Zero(6);

	cout << "The upsized vector v: " << v.transpose() << endl;

	VectorXd vv(8);
	vv << 1, 2, 3, 4, 5, 6, 7, 8;

	cout << "The vector vv: " << vv.transpose() << endl << endl;

	vv.conservativeResizeLike(Eigen::VectorXd::Zero(4));

	cout << "The upsized vector vv: " << vv.transpose() << endl;



	return 0;
}