#include <iostream>
using namespace std;


class Base {
public:
    size_t size() const { return n; }
protected:
    size_t n;
};
class Derived : private Base {  // private继承
public:
    using Base::size;
protected:
    using Base::n;
};

int main(int argc, char const *argv[])
{
    Derived d1;
    cout << d1.size();
    return 0;
}
