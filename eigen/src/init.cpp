#include <iostream>

#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

int main() {

	VectorXd v = (VectorXd(3) << 1.0, 0.0, 9.0).finished();
	// VectorXd v = (VectorXd(3) << 1.0, 0.0, 9.0); // doesn't work

	std::cout << "v: " << v.transpose() << std::endl;

	return 0;
}