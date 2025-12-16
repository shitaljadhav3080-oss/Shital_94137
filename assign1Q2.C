#include <stdio.h>

void printBinary(unsigned int n) {
    int i;

    for (i = 31; i >= 0; i--) {

        unsigned int mask = 1 << i;

        if (n & mask)
            printf("1");
        else
            printf("0");
    }
}

int main() {
    unsigned int num;
    printf("Enter a number: ");
    scanf("%u", &num);

    printf("Binary: ");
    printBinary(num);

    return 0;
}

