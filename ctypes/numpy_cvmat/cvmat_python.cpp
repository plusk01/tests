#include <vector>
#include <iostream>
#include <fstream>
#include <string>

#include <opencv2/core/core.hpp>

using namespace std;

extern "C" void mult_by_three(int rows, int cols, double *data){
    cv::Mat D = cv::Mat(rows, cols, CV_64F, data);

    cout << "hi " << rows << " " << cols << endl;

    cout << D << endl;

    D = D*3.0;
}

/*
    g++ -c -fPIC cvmat_python.cpp -o cvmat_python.o
    g++ -shared -Wl,-soname,cvmat_python_lib.so -o cvmat_python_lib.so cvmat_python.o -lopencv_core
*/