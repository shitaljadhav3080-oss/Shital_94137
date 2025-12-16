#include <stdio.h>

int main() {
    unsigned char b;
    int ones = 0, i;

    printf("Enter a byte (0-255): ");
    scanf("%hhu", &b);

    for (i = 0; i < 8; i++) {
        if (b & (1 << i))
            ones++;
    }

    if (ones % 2 == 0) {
        printf("Even parity.\n");
    } else {
        printf("Odd parity. Setting MSB...\n");
        b = b | 0x80;  
    }

    printf("New byte value = %u\n", b);

    return 0;
}

