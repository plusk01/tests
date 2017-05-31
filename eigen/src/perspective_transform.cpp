#include <iostream>

#include <opencv2/core.hpp>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;


void perspectiveTransform(const Eigen::Matrix3d& T, Eigen::Vector2d& pt) {

}

int main() {
	// Original Points
	Eigen::Vector2d pt1, pt2, pt3, pt4;
	pt1 << 0, 0;
	pt2 << 324, 0;
	pt3 << 324, 223;
	pt4 << 0, 223;

	// Homography (from opencv_tuts/feature_homography)
	Eigen::Matrix3d H;
	H << 0.4282981758083035, -0.1864462995904171, 121.5736081318764,
	     0.002224679716159575, 0.3642515155691183, 162.5073053324869,
	     -0.000244616917046638, -0.0004658386229151473, 1;

	// Using OpenCV's perspectiveTransform with above Homography
	Eigen::Vector2d pt5, pt6, pt7, pt8;
	pt5 << 121.57361, 162.50731;
	pt6 << 282.75198, 177.27846;
	pt7 << 267.81104, 299.26248;
	pt8 << 89.269585, 271.9903;


	// 2D Projective matrix (i.e., Planar Homography) is a 3x3 matrix
	Eigen::Projective2d T(H);

	std::cout << "pt1:                               [" << pt1.transpose() << "]^T" << std::endl;
	std::cout << "pt1.homogeneous:                   [" << pt1.homogeneous().transpose() << "]^T" << std::endl;
	std::cout << "T * pt1.homogeneous:               [" << (T*pt1.homogeneous()).transpose() << "]^T" << std::endl;
	std::cout << "(T * pt1.homogeneous).hnormalized: [" << (T*pt1.homogeneous()).eval().hnormalized().transpose() << "]^T" << std::endl;
	std::cout << "OpenCV perspectiveTransform point: [" << pt5.transpose() << "]^T" << std::endl << std::endl;

	std::cout << "pt2:                               [" << pt2.transpose() << "]^T" << std::endl;
	std::cout << "pt2.homogeneous:                   [" << pt2.homogeneous().transpose() << "]^T" << std::endl;
	std::cout << "T * pt2.homogeneous:               [" << (T*pt2.homogeneous()).transpose() << "]^T" << std::endl;
	std::cout << "(T * pt2.homogeneous).hnormalized: [" << (T*pt2.homogeneous()).eval().hnormalized().transpose() << "]^T" << std::endl;
	std::cout << "OpenCV perspectiveTransform point: [" << pt6.transpose() << "]^T" << std::endl << std::endl;

	std::cout << "pt3:                               [" << pt3.transpose() << "]^T" << std::endl;
	std::cout << "pt3.homogeneous:                   [" << pt3.homogeneous().transpose() << "]^T" << std::endl;
	std::cout << "T * pt3.homogeneous:               [" << (T*pt3.homogeneous()).transpose() << "]^T" << std::endl;
	std::cout << "(T * pt3.homogeneous).hnormalized: [" << (T*pt3.homogeneous()).eval().hnormalized().transpose() << "]^T" << std::endl;
	std::cout << "OpenCV perspectiveTransform point: [" << pt7.transpose() << "]^T" << std::endl << std::endl;

	std::cout << "pt4:                               [" << pt4.transpose() << "]^T" << std::endl;
	std::cout << "pt4.homogeneous:                   [" << pt4.homogeneous().transpose() << "]^T" << std::endl;
	std::cout << "T * pt4.homogeneous:               [" << (T*pt4.homogeneous()).transpose() << "]^T" << std::endl;
	std::cout << "(T * pt4.homogeneous).hnormalized: [" << (T*pt4.homogeneous()).eval().hnormalized().transpose() << "]^T" << std::endl;
	std::cout << "OpenCV perspectiveTransform point: [" << pt8.transpose() << "]^T" << std::endl << std::endl;



	return 0;
}