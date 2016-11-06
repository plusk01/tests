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
	//std::vector<cv::Mat> channels;
	//cv::split(tmp, channels);
	//cv::hconcat(channels, tmp);
	tmp = tmp.reshape(1);
	cout << "tmp.type == " << tmp.type() << " ; tmp.channels == " << tmp.channels() << endl;
	cout << endl << tmp << endl << endl;

	// Add a third column (the focal length) to each row
	cv::Mat pts(measurements.size(), 3, CV_32FC1, cv::Scalar(f));
	cout << pts << endl;
	cout << "pts.channels == " << pts.channels() << endl;
	cout << "pts.cols == " << pts.cols << "; pts.rows == " << pts.rows << endl << endl;

	// copy smaller nx2 mat into bigger nx3 mat
	// It would be better if I could figure out how to place a submatrix shallow copy into a new, bigger matrix
	// then, I wouldn't have to move memory around at the end because it was already worked on. I think.
	tmp.copyTo(pts(cv::Rect(0, 0, tmp.cols, tmp.rows))); // deep copy
	cout << "pts:" << endl << pts << endl;

	cout << "pts.isContinous == " << pts.isContinuous() << endl;
	cout << "pts.channels == " << pts.channels() << endl;
        cout << "pts.cols == " << pts.cols << "; pts.rows == " << pts.rows << endl << endl;

	// test deep vs shallow copy of tmp and pts
	//pts.at<float>(0,0) = 555.123;
	//cout << tmp << endl;


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
	cv::divide(pts, F, pts);
	cout << pts << endl;

	// copy to ell_unit_c vector
	cv::Mat sub_pts = pts(cv::Rect(0,0,pts.cols-1,pts.rows)).reshape(2);
	cout << "sub_pts:" << endl << sub_pts << endl;
	sub_pts.copyTo(ell_unit_c);


	print(measurements);
	print(ell_unit_c);

	return 0;
}
