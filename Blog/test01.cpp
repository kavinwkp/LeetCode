#include <bits/stdc++.h>
using namespace std;

class Base {
public:
    Base(): b(1) {}
    virtual void fun() {}
    int b;
};

class Derived: public Base {
public:
    Derived(): d(2) {}
    int d;
};

int main(int argc, char const *argv[])
{
    int n = 97;
    int *p = &n;
    char *c = reinterpret_cast<char*>(p);
    char *c2 = (char*)p;
    cout << *c << ", " << *c2 << endl;  // a, a

    const int *p2 = &n; // 常量指针
    int *p3 = const_cast<int*>(p2); // 转换为普通指针
    *p3 = 100;  // 可以修改指向的对象
    cout << *p2 << ", " << *p3 << endl; // 100, 100

    Base *b1 = new Derived;
    Base *b2 = new Base;
    Derived *d1 = static_cast<Derived*>(b1);    // 同类转换
    Derived *d2 = static_cast<Derived*>(b2);    // 下行转换，不安全
    cout << d1->d << endl;  // 2
    cout << d2->d << endl;  // 0，基类没有d这个成员，也没报错

    Derived *d3 = dynamic_cast<Derived*>(b1);   // 同类转换
    Derived *d4 = dynamic_cast<Derived*>(b2);   // 下行转换，安全
    cout << d3->d << endl;  // 2
    if (d4 == nullptr) {
        cout << "d4 is nullptr" << endl;    // d4 is nullptr
    }
    else {
        cout << d4->d << endl;
    }

    return 0;
}