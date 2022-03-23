#include <iostream>
using namespace std;

class A {
private:
    A() {}
    A(const A&);
    const A& operator=(const A&);
    string name;
public:
    static A& getInstance();
    string getName() { return name; }
    void setName(string inputName) { name = inputName; }
};

A& A::getInstance() {
    static A a;
    return a;
}

int main(int argc, char const *argv[])
{
    A& a = A::getInstance();
    a.setName("kavin");
    cout << a.getName() << endl;
    cout << A::getInstance().getName() << endl;
    return 0;
}

