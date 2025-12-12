#include <stdio.h>

int main() {
    char c;

    printf("Enter a character: ");
    scanf(" %c", &c);

    if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z')) {
        char result = c ^ 32;
        printf("After XOR with 32: %c\n", result);
    } else {
        printf("Not an alphabet.\n");
    }

    return 0;
}

