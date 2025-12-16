#include <stdio.h>
#include <stdlib.h>

int main(int argc, char *argv[]) {

    if (argc < 2) {
        printf("Please pass a number as command line argument.\n");
        return 1;
    }

    unsigned int num = atoi(argv[1]);
    printf("You entered: %u\n", num);

    return 0;
}

