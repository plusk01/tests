#include <vector>
#include <iostream>
#include <fstream>
#include <string>

#include <opencv2/core/core.hpp>

using namespace std;

void print(std::vector<cv::Point2f> measurements) {
	cout << "measurements = ";

	// http://stackoverflow.com/a/14374550/2392520
	for (auto& elem: measurements) {
		// if the current index is needed:
		auto i = &elem - &measurements[0];

		// any code including continue, break, return
		cout << " " << elem;
	}
	
	cout << endl;
}

int main() {

	std::vector<cv::Point2f> measurements;
	measurements.push_back(cv::Point2f(4,4));
	measurements.push_back(cv::Point2f(25,25));
	measurements.push_back(cv::Point2f(36,36));

	cout << "measurements.size() = " << measurements.size() << endl;

	print(measurements);

	return 0;
}
