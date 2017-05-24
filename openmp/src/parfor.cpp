#include <iostream>
#include <chrono>
#include <math.h>

#include <omp.h>

int main() {

	int a[10000], b[10000], c[10000];

	auto start = std::chrono::high_resolution_clock::now();

	#pragma omp parallel
	#pragma omp for nowait
	for (int i = 0; i < 10000; i++) {
		a[i] = b[i] + c[i];
	}






	//   long double i=0;

	// #pragma omp parallel for reduction(+:i)
	//   for(int t=1; t<30000000; t++){       
	//     long double k=0.7;
	//     for(int n=1; n<16; n++){
	//       i=i+pow(k,n);
	//     }
	//   }

	//   std::cout << i<<"\t";
	  // return 0;

	auto end = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsed = end - start;
	std::cout << "Time: " << elapsed.count() << std::endl;
	
	return 0;
}