#include <stdio.h>
#include <stdint.h>

void printBinary(uint8_t n)
{
    int i;                     
    for (i = 7; i >= 0; i--)   
    {
        printf("%d", (n >> i) & 1);
    }
}

int main()
{
    uint8_t reg;

    // SET bit 4
    reg = 0x2A;
    reg = reg | (1 << 4);
    printf("Set bit 4     : Hex = 0x%X, Binary = ", reg);
    printBinary(reg);
    printf("\n");

    // CLEAR bit 1
    reg = 0x2A;
    reg = reg & ~(1 << 1);
    printf("Clear bit 1   : Hex = 0x%X, Binary = ", reg);
    printBinary(reg);
    printf("\n");

    // TOGGLE bit 5
    reg = 0x2A;
    reg = reg ^ (1 << 5);
    printf("Toggle bit 5  : Hex = 0x%X, Binary = ", reg);
    printBinary(reg);
    printf("\n");

    return 0;
}

