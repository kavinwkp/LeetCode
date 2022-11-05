#include <iostream>
using namespace std;

class Base {
public:
    Base(): mem(0) {}
    Base(const Base& rhs): mem(rhs.mem) {}
private:
    int mem;
};

class Derived : public Base {
public:
    Derived(): Base(), name("") {}
    Derived(const Derived& rhs): Base(rhs), name(rhs.name) {}
private:
    string name;
};

int main(int argc, char const *argv[])
{
    Derived d1;
    Derived d2(d1);
    return 0;
}