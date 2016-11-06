#include <vector>
#include <iostream>
#include <fstream>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

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
        measurements.push_back(cv::Point2f(4,8));
        measurements.push_back(cv::Point2f(25,200));
        measurements.push_back(cv::Point2f(36.5,38.5));

        cout << "measurements.size() = " << measurements.size() << endl;
        print(measurements);

	cv::Mat A(measurements);
	cout << "A.type == " << A.type() << " ; A.channels == " << A.channels() << endl;


	// A is a 2-channel matrix (because it came from Point2f). So we need to split
	// up the channels into separate Mat objects...
	std::vector<cv::Mat> s;
	cv::split(A,s);
	cout << s[0] << endl;
	cout << s[1] << endl;

	// And now concat each of the mats into one, CV_64FC1 mat.
	// hconcat (stack by columns); vconcat (stack by rows)
	cv::Mat B;
	cv::hconcat(s, B);
	cout << endl << endl << B << endl;
	cout << "B.type == " << B.type() << " ; B.channels == " << B.channels() << endl;

	return 0;
}
