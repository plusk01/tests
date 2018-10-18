#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

double width;
double sum;
int num_rects = 1000000000;

void calcPi(void)
{
        float x;
        int i;

        for (i = 0; i < num_rects; i++) {
                x = (i + 0.5) * width;
                sum += 4.0 / (1.0 + x * x);
        }
}

int main(int argc, char **argv)
{
        struct timeval start, end;

        if (argc > 1)
                num_rects = atoi(argv[1]);
        if (num_rects < 100)
                num_rects = 1000000;
        printf("\nnum_rects = %d\n", (int)num_rects);

        gettimeofday(&start, NULL);

        sum = 0.0;
        width = 1.0 / (double)num_rects;

        calcPi();

        gettimeofday(&end, NULL);

        printf("PI = %f\n", sum * width);
        printf("Time : %lf sec\n", (double)(end.tv_sec-start.tv_sec)+(double)(end.tv_usec-start.tv_usec)/1000000.0);

        return 0;
}
