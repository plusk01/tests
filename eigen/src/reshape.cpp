#include <iostream>

#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

// From: http://stackoverflow.com/a/38289479/2392520
Map<const MatrixXd> reshape (const VectorXd& b, const uint n, const uint m) {
    return Map<const MatrixXd>(b.data(), n, m);
}

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

	int m = v.size()/2;
	MatrixXd A = Map<MatrixXd>(v.data(), m, 2);

	// MatrixXd A = reshape(v, m, 2);

	cout << "The (3, 2) matrix A:" << endl;
	cout << A << endl;

	return 0;
}