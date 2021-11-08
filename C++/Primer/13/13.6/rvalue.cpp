#include <iostream>

using namespace std;

int num = 10;

int& func1(int& num) {
    num = num * num;
    return num;
}

int func2(int num) {
    return num * num;
}

int main(int argc, char const *argv[])
{
    // int i = 10;
    // int &r1 = i;        // OK 左值引用
    // // int &&r2 = i;       // Err 不能将右值引用绑定到左值
    // // int &r3 = i * 10;   // Err i*42是一个右值
    // const int &r3 = i * 10; // OK 可以将const引用绑定到右值
    // cout << r3 << endl;
    // int &&r4 = i * 10;      // OK 右值引用
    // cout << r4 << endl;

    int a = 5;
    int &ref1 = func1(a);
    cout << ref1 << endl;

    int b = 4;
    int &&ref2 = func2(b);
    cout << ref2 << endl;

    const int &ref3 = func2(b);
    cout << ref3 << endl;

    int &&r1 = 10;
    // int &&r2 = r1;
    int &&r2 = std::move(r1);
    cout << r1 << endl;

    return 0;
}
