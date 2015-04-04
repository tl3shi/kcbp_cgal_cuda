#include <time.h>
#include <iostream>
#include <vector>
using namespace std;
int main111()
{
    clock_t s = clock();
    int n = 1e4;
    float sum = 0;
    for(int i = 0; i < n-1;  i++)
        for(int j = i+1; j < n; j++)
            sum += (i-j) * 1.0 / j;
    printf("sum = %.2f\n", sum);
    s = clock() - s;
    printf("time = %.2f", s * 1.0);
    return 0;
}