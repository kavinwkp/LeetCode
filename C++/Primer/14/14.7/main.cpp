#include <iostream>

using namespace std;

class Test {
public:
    Test() = default;
    Test(string str): p(&str), curr(0) {}
    char& operator*() const {
        return (*p)[curr];  // (*p)是对象所指的vector
    }
    char* operator->() const {
        return &this->operator*();
    }

private:
    string* p;
    size_t curr;
};

int main(int argc, char const *argv[])
{
    Test t("hello");
    t.operator->
    return 0;
}
