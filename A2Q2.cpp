#include <stdio.h>
#include <stdint.h>

int main()
{
    uint8_t status = 0x08;   // 0000 1000

    if (status & (1 << 3))
    {
        printf("Bit 3 is SET\n");
    }
    else
    {
        printf("Bit 3 is CLEAR\n");
    }

    return 0;
}

