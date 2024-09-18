#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define BLOCK_SIZE (1024 * 1024)

// Function to report differences at the bit level
void report_bit_difference(size_t start_byte, int start_bit, size_t end_byte, int end_bit) {
    printf("Difference from byte %zu bit %d to byte %zu bit %d.\n", start_byte, start_bit, end_byte, end_bit);
}

// Function to perform detailed bit-level comparison within a block
void detailed_comparison(const uint8_t *ptr1, const uint8_t *ptr2, size_t start, size_t end) {
    int in_diff = 0;
    size_t diff_start_byte = 0;
    int diff_start_bit = 0;

    for (size_t i = start; i <= end; i++) {
        uint8_t diff = ptr1[i] ^ ptr2[i];  // XOR to find differing bits
        if (diff != 0) {  // Check if there are any differing bits
            for (int j = 0; j < 8; j++) {  // Check each bit in the byte
                if (diff & (1 << j)) {  // Check if the j-th bit is different
                    if (!in_diff) {
                        diff_start_byte = i;
                        diff_start_bit = j;
                        in_diff = 1;
                    }
                } else {
                    if (in_diff) {
                        report_bit_difference(diff_start_byte, diff_start_bit, i, j - 1);
                        in_diff = 0;
                    }
                }
            }
        } else {
            if (in_diff) {
                report_bit_difference(diff_start_byte, diff_start_bit, i - 1, 7);
                in_diff = 0;
            }
        }
    }

    // If the last bit examined was different and the block is still open
    if (in_diff) {
        report_bit_difference(diff_start_byte, diff_start_bit, end, 7);
    }
}

// Function to find differences using block comparison
void find_differences(const uint8_t *ptr1, const uint8_t *ptr2, size_t num) {
    size_t blocks = num / BLOCK_SIZE;
    for (size_t i = 0; i < blocks; i++) {
        size_t start = i * BLOCK_SIZE;
        size_t end = start + BLOCK_SIZE - 1;
        if (memcmp(ptr1 + start, ptr2 + start, BLOCK_SIZE) != 0) {
            detailed_comparison(ptr1, ptr2, start, end);
        }
    }

    size_t remaining_start = blocks * BLOCK_SIZE;
    size_t remaining_bytes = num % BLOCK_SIZE;
    if (remaining_bytes > 0) {
        if (memcmp(ptr1 + remaining_start, ptr2 + remaining_start, remaining_bytes) != 0) {
            detailed_comparison(ptr1, ptr2, remaining_start, num - 1);
        }
    }
}

void fill_random_data(uint8_t *data, size_t size) {
    for (size_t i = 0; i < size; i++) {
        data[i] = rand() % 256;  // Random byte value between 0 and 255
    }
}

// Function to introduce specific multi-byte bit differences
void introduce_bit_differences(uint8_t *data1, uint8_t *data2, size_t size) {
    // Ensure to copy data1 to data2 first to start with identical copies
    memcpy(data2, data1, size);

    // Introduce a multi-byte bit difference from byte 250 to byte 255
    data2[249] ^= 1 << 7;
    data2[249] ^= 1 << 6;
    for (size_t i = 250; i <= 255 && i < size; i++) {
        data2[i] ^= 0xFF;
    }
    data2[256] ^= 1 << 0;

    // Introduce another multi-byte bit difference from byte 200000 to byte 200005
    for (size_t i = 200000; i <= 200005 && i < size; i++) {
        data2[i] ^= 1 << 6;  // Flip the 6th bit in each byte from 200000 to 200005
        data2[i] ^= 1 << 5;
        data2[i] ^= 1 << 4;
    }
}

void benchmark(size_t size) {
    uint8_t *data1 = malloc(size);
    uint8_t *data2 = malloc(size);

    if (data1 == NULL || data2 == NULL) {
        printf("Memory allocation failed.\n");
        return;
    }

    // Fill arrays with random data
    srand(1);
    fill_random_data(data1, size);

    memcpy(data2, data1, size);
    introduce_bit_differences(data1, data2, size);

    for (int i = 0; i < 2; i++) {
        // Timing the difference finding
        clock_t start_time = clock();
        find_differences(data1, data2, size);
        clock_t end_time = clock();

        double time_spent = (double)(end_time - start_time) / CLOCKS_PER_SEC;
        printf("Time spent: %.6f seconds\n", time_spent);
    }

    free(data1);
    free(data2);
}

int main() {
    size_t size = 1024 * 1024 * 10;
    benchmark(size);

    return 0;
}