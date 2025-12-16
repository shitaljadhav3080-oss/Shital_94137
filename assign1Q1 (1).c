#include <stdio.h>

int countOnes(int n) {
    int count = 0;

    while (n > 0) {
        if (n & 1) {      
            count++;
        }
        n = n >> 1;       
    }

    return count;
}

int main() {
    int number;
    printf("Enter a number: ");
    scanf("%d", &number);

    printf("Number of 1 bits = %d\n", countOnes(number));
    return 0;
}

