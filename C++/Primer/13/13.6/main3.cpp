#include <iostream>

using namespace std;

class Person
{
private:
    string *ptr;
    int age;

public:
    Person(const string &s = string()) : ptr(new string(s)), age(0) {}
    Person(const Person &p) : ptr(new string(*p.ptr)), age(p.age) {
        cout << "copy constructor" << endl;
    }    // 拷贝构造函数

    Person(Person &&p) noexcept // 移动构造函数
        : ptr(p.ptr), age(p.age)
    {
        cout << "move constructor" << endl;
        p.ptr = nullptr; // 源对象的指针置空
    }

    Person& operator=(const Person&) &;

    ~Person() { delete ptr; }
};

Person& Person::operator=(const Person& rhs) &
{
    auto newptr = new string(*rhs.ptr); // 拷贝资源
    delete ptr;     // 释放当前对象的资源
    ptr = newptr;   // 指针指向新的资源
    age = rhs.age;
    return *this;
}

Person func() {
    Person *p = new Person("kavin");
    return *p;
}

int main(int argc, char const *argv[])
{
    Person p1("kavin"), p2;
    // p2 = p1;
    p2 = func();
    return 0;
}
