#include <iostream>
using namespace std;


class Base
{
    friend class Pal;
protected:
    int prot_mem;
};

class Sneaky : public Base {
    friend void clobber(Sneaky&);
    friend void clobber(Base&);
    int j = 0;
};

void clobber(Sneaky &s) {
    s.j = s.prot_mem = 0;
}

// void clobber(Base &b) {
//     b.prot_mem = 0;
// }

class Pal {
public:
    int f(Base b) { return b.prot_mem; }
    // int f2(Sneaky s) { return s.j; }
    int f3(Sneaky s) { return s.prot_mem; }
};

class D2 : public Pal {
public:
    int mem(Base b) { return b.prot_mem; }
};

int main(int argc, char const *argv[])
{
    // Base b;
    Pal p;
    cout << p.f3(Sneaky());
    return 0;
}
