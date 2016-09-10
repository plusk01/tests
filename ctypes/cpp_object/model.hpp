#ifndef MODEL_H
#define MODEL_H

#include "opencv2/core/core.hpp"

class Model {

public:
    Model(void): T(0), A(cv::Mat::zeros(8,1,CV_64F)), width(0.0f), height(0.0f) {};
    ~Model(void) {};

    cv::Mat A;
    int T;
    double rho;
    std::vector<int> CS;
    float width;
    float height;

};

#endif //MODEL_H
