#include <iostream>
#include <chrono>

#include <omp.h>

int main() {

	int a[10000], b[10000], c[10000];

	auto start = std::chrono::high_resolution_clock::now();

	#pragma omp parallel
	#pragma omp for nowait
	for (int i = 0; i < 10000; i++) {
		a[i] = b[i] + c[i];
	}

	auto end = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsed = end - start;
	std::cout << "Time: " << elapsed.count() << std::endl;
	
	return 0;
}