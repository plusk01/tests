// vector_python.cpp
#include <vector>
#include <iostream>
#include <fstream>
#include <string>

#include <opencv2/core/core.hpp>
#include <boost/shared_ptr.hpp>

#include "model.hpp"

using namespace std;

extern "C" {
    Model* new_model() {
        return new Model();
    }
    void delete_model(Model* m){
        cout << "destructor called in C++ for (model) " << m << endl;
        delete m;
    }

    float model_get_height(Model* m) {
        return m->height;
    }
    void model_set_height(Model* m, float height) {
        m->height = height;
    }

    float model_get_width(Model* m) {
        return m->width;
    }
    void model_set_width(Model* m, float width) {
        m->width = width;
    }


    double model_get_rho(Model* m) {
        return m->rho;
    }
    void model_set_rho(Model* m, double rho) {
        m->rho = rho;
    }


    int model_get_T(Model* m) {
        return m->T;
    }
    void model_set_T(Model* m, int T) {
        m->T = T;
    }


    vector<int>* model_get_CS(Model* m) {
        return &(m->CS);
    }

}

/*
    g++ -c -fPIC model_python.cpp -o model_python.o
    g++ -shared -Wl,-soname,model_python_lib.so -o model_python_lib.so model_python.o -lopencv_core
*/