#include <iostream>
#include <cmath>

// https://stackoverflow.com/a/50210009/2392520
#define _GNU_SOURCE
#include <assert.h>
#include <sched.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>





void print_affinity() {
    cpu_set_t mask;
    long nproc, i;

    if (sched_getaffinity(0, sizeof(cpu_set_t), &mask) == -1) {
        perror("sched_getaffinity");
        assert(false);
    } else {
        nproc = sysconf(_SC_NPROCESSORS_ONLN);
        printf("sched_getaffinity = ");
        for (i = 0; i < nproc; i++) {
            printf("%d ", CPU_ISSET(i, &mask));
        }
        printf("\n");
    }
}




int main() {


    #pragma omp parallel
    {
    // Code inside this region runs in parallel.
    printf("Hello!\t");
    printf("sched_getcpu = %d\n", sched_getcpu());
    }

    #pragma omp for
    for(int n=0; n<10; ++n)
    {
      printf(" %d", n);
    }
    printf(".\n");

    printf("sched_getcpu = %d\n", sched_getcpu());
    print_affinity();


    return 0;
}