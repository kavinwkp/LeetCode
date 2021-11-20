#include <stdio.h>

int main(int argc, char const *argv[])
{
    short int a = -12345;
    unsigned short b = (unsigned short)a;
    printf("a = %d, b = %u", a, b);
    return 0;
}
a = -12345, b = 53191
