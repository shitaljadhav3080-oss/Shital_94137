#include <stdio.h>
#include <stdint.h>

int main()
{
    uint8_t reg = 0xAA;   // 10101010

    printf("Register before = 0x%X\n", reg);

    // Read bits 2 to 4
    uint8_t read_bits = (reg & 0x1C) >> 2;
    printf("Bits 2 to 4 value = 0b%03b\n", read_bits);

    // Write 0b011 into bits 2 to 4
    reg = reg & ~0x1C;           // Clear bits 2–4
    reg = reg | (0x03 << 2);     // Set new value

    printf("Register after  = 0x%X\n", reg);

    return 0;
}

