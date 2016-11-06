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
	cout << A << endl << endl;


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

	// What about using reshape?
	cv::Mat C = cv::Mat(measurements).reshape(1);
	cout << "C.type == " << C.type() << " ; C.channels == " << C.channels() << endl;
	cout << C << endl << endl;

	C = C.mul(C);

	// Now, move back to a vector<Point2f>
	//measurements.clear();
	print(measurements); // note that measurements is already linked to C, so the
	// following (in this case) is actually unneccessary
	cout << "C.isContinouts == " << C.isContinuous() << endl;
	// Make a 2 channel matrix so we can put in a Point2f vector
	// I don't think this method matters if C is continous or not...
	C = C.reshape(2);
	cout << "C.type == " << C.type() << " ; C.channels == " << C.channels() << endl;
	cout << "C.rows == " << C.rows << " ; C.cols == " << C.cols << endl;
        cout << C << endl << endl;
	//C = C.clone(); // why is this necessary?
	// it seems like the above is neccessary when I do `measurements.clear`. I assume that
	// this is a memory thing since the original mat came from measurements anyway.
	C.copyTo(measurements);

	print(measurements);

	cout << "C, one more time for good measure:" << endl;
	cout << C << endl;


	// example of mat.copyTo(vec) starting with a constant mat
	cout << endl << endl << endl << endl;
	std::vector<cv::Point2f> tVec;
	cv::Mat aMat = (cv::Mat_<float>(3, 2) << 123,444, 11,6, 50,13);
	aMat = aMat.reshape(2);
	cout << "aMat.type == " << aMat.type() << " ; aMat.channels == " << aMat.channels() << endl;
	cout << aMat << endl;
	aMat.copyTo(tVec);
	print(tVec);

	return 0;
}
