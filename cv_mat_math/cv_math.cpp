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
	measurements.push_back(cv::Point2f(36.5,36.5));

	cout << "measurements.size() = " << measurements.size() << endl;
	print(measurements);

	// focal length
	double f = 2.6;

	// Is it not working because I am starting from vector<Point2f> ?
	cv::Mat A(2,1,CV_64FC2,cv::Scalar(1,2));
	cout << A << endl;
	cout << A.channels() << endl;

	cv::Mat B(2,2,CV_64FC1);
	A.convertTo(B,CV_32FC1);
	cout << B << endl;
	cout << B.channels() << endl << endl << endl;
	// it still didn't work :(




	// turn vector of points into a mat
	cv::Mat pts_nby1c2(measurements);
	cv::Mat tmp;
	pts_nby1c2.convertTo(tmp, CV_64FC1, 0.5);

	cout << tmp << endl;
	cout << tmp.channels() << endl;

	cout << pts_nby1c2 << endl;
	cout << pts_nby1c2.channels() << endl;
	cout << pts_nby1c2.rows << ", " << pts_nby1c2.cols << endl;


	// Add a third column (the focal length) to each row
	cv::Mat pts(measurements.size(), 1, CV_64FC3, cv::Scalar(0,0,f));
	cout << pts << endl;
	cout << pts.channels() << endl;

	tmp.copyTo(pts(cv::Rect(0, 0, pts.cols, pts.rows)));
	cout << pts << endl;

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
