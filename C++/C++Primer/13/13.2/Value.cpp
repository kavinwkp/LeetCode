#include <iostream>

using namespace std;

class Person
{
private:
    string *ptr;
    int age;

public:
    Person(const string &s = string()) : ptr(new string(s)), age(0) {}
    Person(const Person &p) : ptr(new string(*p.ptr)), age(p.age) {}
    Person& operator=(const Person& rhs) {
        cout << "copy assignment" << endl;
        auto newptr = new string(*rhs.ptr);
        delete ptr;     // 释放当前对象的资源
        ptr = newptr;   // 指针指向新的资源
        age = rhs.age;
        return *this;
    }
    ~Person() { delete ptr; }
};

int main(int argc, char const *argv[])
{
    Person p1("kavin");
    Person p2;
    p2 = p1;
    return 0;
}
