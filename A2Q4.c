#include <stdio.h>
#include <stdint.h>

int main()
{
    uint8_t value = 5;

    uint8_t left_result = value << 2;
    uint8_t right_result = value >> 1;

    printf("Original value = %d\n", value);
    printf("Left shift by 2 = %d\n", left_result);
    printf("Right shift by 1 = %d\n", right_result);

    return 0;
}

