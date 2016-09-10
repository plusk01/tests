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
    boost::shared_ptr<Model> new_model() {
        boost::shared_ptr<Model> m(new Model());
        return m;
    }
    void delete_model(boost::shared_ptr<Model> m){
        cout << "destructor called in C++ for (model) " << m << endl;
        // delete m;
    }

    float model_get_height(boost::shared_ptr<Model> m) {
        return m->height;
    }
    void model_set_height(boost::shared_ptr<Model> m, float height) {
        m->height = height;
    }

    float model_get_width(boost::shared_ptr<Model> m) {
        return m->width;
    }
    void model_set_width(boost::shared_ptr<Model> m, float width) {
        m->width = width;
    }


    double model_get_rho(boost::shared_ptr<Model> m) {
        return m->rho;
    }
    void model_set_rho(boost::shared_ptr<Model> m, double rho) {
        m->rho = rho;
    }


    int model_get_T(boost::shared_ptr<Model> m) {
        return m->T;
    }
    void model_set_T(boost::shared_ptr<Model> m, int T) {
        m->T = T;
    }


    vector<int>* model_get_CS(boost::shared_ptr<Model> m) {
        return &(m->CS);
    }

}

/*
    g++ -c -fPIC model_python.cpp -o model_python.o
    g++ -shared -Wl,-soname,model_python_lib.so -o model_python_lib.so model_python.o -lopencv_core
*/