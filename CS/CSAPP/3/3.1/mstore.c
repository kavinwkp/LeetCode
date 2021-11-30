long mul2(long, long);

void mulstore(long x, long y, long *dest) {
    long t = mul2(x, y);
    *dest = t;
}