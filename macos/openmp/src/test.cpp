#include <iostream>
#include <omp.h>
int main() {
    #pragma omp parallel num_threads(4)
    {
        #pragma omp critical
        std::cout << "tid = " << omp_get_thread_num() << std::endl;
    }
}
