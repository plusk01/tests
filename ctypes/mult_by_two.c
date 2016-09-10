#include <stdio.h>

void mult_by_two(const double *indatav, size_t size, double *outdatav) {
    
    // loop through each element of the array and multiply by two
    size_t i;
    for (int i = 0; i < size; ++i) {
        outdatav[i] = indatav[i] * 2.0;
    }
}

/*
    Create a shared library (from C)
    gcc -fPIC -shared -o mult_by_two.so mult_by_two.c
*/