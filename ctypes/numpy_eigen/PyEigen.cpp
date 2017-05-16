#include <vector>
#include <iostream>
#include <fstream>
#include <string>

#include <Eigen/Dense>

using namespace std;

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> RowMajorMatrix;
typedef Eigen::Map<RowMajorMatrix> RowMajorMap;

extern "C" void mult_by_three(int rows, int cols, double *data) {
    Eigen::MatrixXd D = RowMajorMap(data, rows, cols);

    cout << "\033[1;33m";

    cout << "hi " << rows << " " << cols << endl;

    cout << D << endl;

    D = D*3.0;

    cout << endl << "now multiplied:" << endl;

    cout << D << endl << endl;

    cout << "Array: ";
    for (int i=0; i<rows*cols; i++)
        cout << data[i] << " ";
    cout << endl << endl;

    // data = D.data();

    // Copy the multiplied data into the original buffer
    // so it will show up in numpy
    memcpy(data, D.data(), rows*cols*sizeof(*data));

    cout << "D.data(): " << D.data() << "\tdata: " << data << endl << endl;

    cout << "\033[0m";
}

extern "C" void vectorize(int rows, int cols, double *data) {
    // cv::Mat tmp = cv::Mat(rows, cols, CV_64F, data).t();


    // cout << tmp << endl;


    // const double *p = tmp.ptr<double>(0);
    // std::vector<double> vec(p, p + tmp.cols);

    // unsigned int i;
    // for (i = 0; i < vec.size(); ++i) {
    //     cout << vec[i] << endl;
    // }
}

extern "C" void create_vector_point2f(int rows, int cols, double *data) {
    // cv::Mat tmp = cv::Mat(rows, cols, CV_64F, data);

    // std::vector<cv::Point2f> vec;

    // int i;
    // for (i = 0; i < tmp.rows; ++i) {
    //     cv::Point2f p(tmp.at<double>(i, 0), tmp.at<double>(i, 1));
    //     vec.push_back(p);
    // }

    // for (i=0; i < tmp.rows; ++i) {
    //     cout << vec[i] << endl;
    // }
}

extern "C" void fill_with_randn(int rows, int cols, double *data) {
    // cv::Mat rand_mat = cv::Mat(rows, cols, CV_64F);

    // cv::Mat mean = cv::Mat::zeros(1,1, CV_64F)*.5;
    // cv::Mat sigma = cv::Mat::ones(1,1, CV_64F);
    // cv::RNG rng( cv::getTickCount() );

    // rng.fill(rand_mat, cv::RNG::NORMAL, mean, sigma);

    // // Copy our rand_mat to a tmp mat that is tied to numpy with *data
    // cv::Mat tmp = cv::Mat(rows, cols, CV_64F, data);
    // // tmp = rand_mat.clone();
    // rand_mat.copyTo(tmp);
    // // data = (double*) rand_mat.data;

    // // http://docs.opencv.org/2.4/modules/core/doc/basic_structures.html?highlight=clone#mat-clone

    // cout << "From C++, the random values are:" << endl;
    // cout << rand_mat << endl;
}

/*
    g++ -c -fPIC cvmat_python.cpp -o cvmat_python.o
    g++ -shared -Wl,-soname,cvmat_python_lib.so -o cvmat_python_lib.so cvmat_python.o -lopencv_core
*/