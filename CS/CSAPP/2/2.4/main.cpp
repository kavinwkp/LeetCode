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
    printf("%.30f\n", *y);
    show_float(*y);
    // int x2 = 0x3f7fffff;
    // float *y2 = (float*)&x2;
    // printf("%.30f\n", *y2);
    // show_float(*y2);
    float y2 = 0.99999998;
    show_float(y2);
    if (*y == y2) printf("Yes\n");
    else printf("No\n");
    return 0;
}
// 1.000000
//  00 00 80 3f
// 0.999999940395355224609375000000
//  ff ff 7f 3f