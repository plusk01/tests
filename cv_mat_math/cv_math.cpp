#include <vector>
#include <iostream>
#include <fstream>
#include <string>

#include <opencv2/core/core.hpp>

using namespace std;

void print(std::vector<cv::Point2f> measurements) {
	cout << "measurements = ";

	// http://stackoverflow.com/a/14374550/2392520
	for (auto&& elem: measurements) {
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

	double f = 2.6;
	std::vector<cv::Point2f> ell_unit_c;

	// Do some math
	std::transform(measurements.begin(), measurements.end(), std::back_inserter(ell_unit_c), [f](cv::Point2f p) {
		cv::Point2f tmp;

		double F = std::sqrt(p.x*p.x + p.y*p.y + f*f);	

		cout << "F: " << F << endl;

		return p/F;
	});

	print(measurements);
	print(ell_unit_c);

	return 0;
}
