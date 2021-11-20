#include <stdio.h>

typedef unsigned char* byte_pointer;

void show_bytes(byte_pointer start, int len) {
    for (int i = 0; i < len; i++)
        printf(" %.2x", start[i]);
    printf("\n");
}

void show_int(int x) {
    show_bytes((byte_pointer)& x, sizeof(x));
}
void show_pointer(void *x) {
    show_bytes((byte_pointer) &x, sizeof(void *));
}
int main(int argc, char const *argv[])
{
    int x = 12345;
    show_pointer(&x);
    return 0;
}
