// vector_python.cpp
#include <vector>
#include <iostream>
#include <fstream>
#include <string>

using namespace std;

extern "C" vector<int>* mult_by_three(vector<int>* v){
    vector<int>* v2 = new vector<int>;

    for (int i = 0; i < v->size(); ++i) {
        v2->push_back(v->at(i) * 3);
    }

    return v2;
}

extern "C" {
    vector<int>* new_vector(){
        return new vector<int>;
    }
    void delete_vector(vector<int>* v){
        cout << "destructor called in C++ for " << v << endl;
        delete v;
    }
    int vector_size(vector<int>* v){
        return v->size();
    }
    int vector_get(vector<int>* v, int i){
        return v->at(i);
    }
    void vector_push_back(vector<int>* v, int i){
        v->push_back(i);
    }
}

/*
    g++ -c -fPIC vector_python.cpp -o vector_python.o
    g++ -shared -Wl,-soname,vector_python_lib.so -o vector_python_lib.so vector_python.o
*/