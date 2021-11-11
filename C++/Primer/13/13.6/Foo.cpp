#include <iostream>
#include <vector>
#include <algorithm>
using namespace std;

class Foo
{
private:
    vector<int> data;
public:
    Foo sorted() &&;
    Foo sorted() const &;
};

Foo Foo::sorted() && 
{
    sort(data.begin(), data.end());
    return *this;
}

Foo Foo::sorted() const & 
{
    cout << "left value" << endl;
    Foo ret(*this);
    sort(ret.data.begin(), ret.data.end());
    return ret;
}

int main(int argc, char const *argv[])
{
    Foo r1;
    r1.sorted();
    return 0;
}
