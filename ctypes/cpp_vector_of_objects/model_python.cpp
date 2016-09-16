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
        // cv::Ptr<Model> m(new Model());
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

// extern "C" vector<boost::shared_ptr<Model>>* mult_by_three(vector<boost::shared_ptr<Model>>* v){
//     vector<int>* v2 = new vector<int>;

//     for (int i = 0; i < v->size(); ++i) {
//         v2->push_back(v->at(i) * 3);
//     }

//     return v2;
// }

extern "C" {
    vector<Model*>* new_vector(){
        return new vector<Model*>;
    }
    void delete_vector(vector<Model*>* v){
        cout << "destructor called in C++ for " << v << endl;
        delete v;
    }
    int vector_size(vector<Model*>* v){
        return v->size();
    }
    Model* vector_get(vector<Model*>* v, int i){
        return v->at(i);
    }
    void vector_push_back(vector<Model*>* v, Model* m){
        v->push_back(m);
    }
}


extern "C" {
    vector<int>* new_int_vector(){
        return new vector<int>;
    }
    void delete_int_vector(vector<int>* v){
        cout << "destructor called in C++ for " << v << endl;
        delete v;
    }
    int int_vector_size(vector<int>* v){
        return v->size();
    }
    int int_vector_get(vector<int>* v, int i){
        return v->at(i);
    }
    void int_vector_push_back(vector<int>* v, int i){
        v->push_back(i);
    }
}



/*
    g++ -Wall -c -fPIC model_python.cpp -o model_python.o
    g++ -Wall -shared -Wl,-soname,lib.so -o lib.so model_python.o -lopencv_core
*/