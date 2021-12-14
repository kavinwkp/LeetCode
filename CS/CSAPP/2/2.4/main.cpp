#include <stdio.h>

typedef unsigned char* byte_pointer;

void show_bytes(byte_pointer start, int len) {
    for (int i = 0; i < len; i++)
        printf(" %.2x", start[i]);   // 两位十六进制
    printf("\n");
}

void show_int(int x) {
    show_bytes((byte_pointer)& x, sizeof(x));
}

void show_float(float x) {
    show_bytes((byte_pointer)& x, sizeof(x));
}

int main() {
    // int x = 0x3f800000;
    // show_int(x);
    // printf("%d\n", x);
    // float y = 1.0;
    // show_float(y);
    int x = 0x3f800000;
    float *y = (float*)&x;
    printf("%f\n", *y);
    show_float(*y);
    return 0;
}
//  67 45 23 01