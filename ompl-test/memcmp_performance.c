#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define SIZE (1024 * 1024 * 1024)

int main() {
    char *data1 = malloc(SIZE);
    char *data2 = malloc(SIZE);

    // Initialize data
    memset(data1, 'A', SIZE);
    memset(data2, 'A', SIZE);

    // Introduce a difference
    data2[SIZE - 1] = 'B';

    // Benchmark memcmp
    clock_t start = clock();
    int cmp = memcmp(data1, data2, SIZE);
    clock_t end = clock();

    double time_taken = (double)(end - start) / CLOCKS_PER_SEC;
    printf("memcmp took %f seconds\n", time_taken);
    printf("Comparison result: %d\n", cmp);

    free(data1);
    free(data2);

    return 0;
}