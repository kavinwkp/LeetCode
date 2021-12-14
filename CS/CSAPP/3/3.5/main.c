#include <stdio.h>

int comp(long a, long b) {
    return a < b;
}

int main(int argc, char const *argv[])
{
    long a = 10;
    long b = 20;
    printf("%d\n", comp(a, b));
    return 0;
}
