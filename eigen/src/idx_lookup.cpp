#include <iostream>

#include <Eigen/Dense>

std::vector<int> get_index_map(Eigen::MatrixXd C) {
	std::vector<int> idx_map;
	idx_map.reserve(C.rows());

	// I should loop in column major order (varying the row the quickest)
	// (Col-Major order: https://stackoverflow.com/a/16286562/2392520)
	// But because of my index finding algorithm I'll do the inefficient order.
	// Of course, my 'C' matrices are going to be pretty small (less than 10x10)
	// so it probably fits in L1 cache and won't make a huge difference.
	for (int r=0; r<C.rows(); r++) {
		for (int c=0; c<C.cols(); c++) {
			
			if (C(r,c) == 1)
				idx_map.push_back(c);

			std::cout << C(r,c) << " ";
		}

		std::cout << std::endl;
	}

	std::cout << "\nidx_map: \n";
	for (int i=0; i<idx_map.size(); i++)
		std::cout << idx_map[i] << " ";
	std::cout << std::endl;

	return idx_map;

}

int main() {

	Eigen::VectorXd xhat = Eigen::VectorXd::Zero(4);
	Eigen::VectorXd y    = Eigen::VectorXd::Random(2);
	
	// Define a C matrix, which relates state space to measurement space
	Eigen::MatrixXd C(2, 4);
	C << 0, 0, 1, 0,
	     1, 0, 0, 0;

	// Find the index map to show us how to put things in measurement space
	// back into the correct position in xhat
	std::vector<int> idx_map = get_index_map(C);

	std::cout << std::endl;
	std::cout << "xhat:    " << xhat.transpose() << std::endl;
	std::cout << "y:      " << y.transpose() << std::endl;

	// Do it!
	for (int i=0; i<idx_map.size(); i++)
		xhat(idx_map[i]) = y(i);

	std::cout << "xhat:   " << xhat.transpose() << std::endl;
	std::cout << "C*xhat: " << (C*xhat).transpose() << std::endl;


	return 0;
}