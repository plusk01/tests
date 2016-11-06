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
	measurements.push_back(cv::Point2f(4,16));
	measurements.push_back(cv::Point2f(25,100));
	measurements.push_back(cv::Point2f(36.5,123));

	cout << "measurements.size() = " << measurements.size() << endl;
	print(measurements);

	// focal length
	float f = 2.6;

	// Convert Point2f vector to a Mat (CV_32FC2)
	cv::Mat tmp(measurements);
	cout << "tmp.type == " << tmp.type() << " ; tmp.channels == " << tmp.channels() << endl;

	// Convert A (CV_32FC2) to B (CV_32FC1) (could I do cv::Mat::reshape instead?)
	std::vector<cv::Mat> channels;
	cv::split(tmp, channels);
	cv::hconcat(channels, tmp);
	cout << "tmp.type == " << tmp.type() << " ; tmp.channels == " << tmp.channels() << endl;
	cout << endl << tmp << endl << endl;

	// Add a third column (the focal length) to each row
	cv::Mat pts(measurements.size(), 3, CV_32FC1, cv::Scalar(f));
	cout << pts << endl;
	cout << "pts.channels == " << pts.channels() << endl;
	cout << "pts.cols == " << pts.cols << "; pts.rows == " << pts.rows << endl << endl;

	// copy smaller nx2 mat into bigger nx3 mat
	tmp.copyTo(pts(cv::Rect(0, 0, tmp.cols, tmp.rows)));
	cout << "pts:" << endl << pts << endl;

	// Do some math
	cv::Mat F = pts.mul(pts);
	cv::reduce(F, F, 1, CV_REDUCE_SUM); // 1 is the dimension to sum along
	cv::sqrt(F, F); // now matrix is nx1

	// expand matrix back to nx3 so we can divide
	F = cv::repeat(F, 1, 3);

	cout << endl << endl << "F:" << endl <<F << endl << endl;

	// divide and create ell_unit_c
	std::vector<cv::Point2f> ell_unit_c;
	cv::Mat pts_unit_c;
	cv::divide(pts, F, pts_unit_c);
	
	cout << pts_unit_c << endl;

	print(measurements);
	print(ell_unit_c);

	return 0;
}
