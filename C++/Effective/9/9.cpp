#include <iostream>
#include <vector>
#include <optional>
using namespace std;

class Base {
public:
    Base(): mem(0) {
        cout << "Base::Base" << endl;
        // fun();
    }
    virtual void fun() { cout << "Base::fun" << endl; }
private:
    int mem;
};

class Derived : public Base {
public:
    Derived(): Base() {
        cout << "Derived::Derived" << endl;
        // fun();
    }
    virtual void fun() override {
        cout << "Derived::fun" << endl;
    }

    void func() {
        cout << "non-virtual fun" << endl;
    }
};

class A {
public:
    A() {
        cout << "default constructor" << endl;
    }
    A(int n) : num(n) {
        cout << "A(int) constructor" << endl;
    }
    A(const A& rhs) : num(rhs.num) {
        cout << "copy constructor" << endl;
    }
    A& operator=(const A& rhs) {
        num = rhs.num;
        cout << "assignment constructor" << endl;
        return *this;
    }
private:
    int num;
};


int main(int argc, char const *argv[])
{
    return 0;
}