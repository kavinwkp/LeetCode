#include <iostream>
#include <vector>
using namespace std;

class Foo {
public:
    Foo() {}
    Foo(int n): num(n) {
        cout << "constructor" << endl;
    }
    Foo(const Foo& rhs): num(rhs.num) {
        cout << "copy constructor" << endl;
    }
    Foo& operator=(const Foo& rhs) {
        num = rhs.num;
        cout << "assignment constructor" << endl;
        return *this;
    }
private:
    int num;
};


int main(int argc, char const *argv[])
{
    vector<Foo> vec;
    // vec.push_back(Foo(1));
    vec.emplace_back(1);
    return 0;
}
