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
    Person &operator=(const Person &rhs)
    {
        cout << "copy assignment" << endl;
        auto newptr = new string(*rhs.ptr);
        delete ptr;   // 释放当前对象的资源
        ptr = newptr; // 指针指向新的资源
        age = rhs.age;
        return *this;
    }

    Person(Person &&p) noexcept // 移动操作不应抛出异常
        : ptr(p.ptr), age(p.age)
    {
        cout << "move constructor" << endl;
        p.ptr = nullptr; // 源对象的指针置空
    }

    Person& operator=(Person &&rhs) noexcept
    {
        cout << "move copy assignment" << endl;
        if (this != &rhs) {
            delete ptr;
            ptr = rhs.ptr;
            age = rhs.age;
            rhs.ptr = nullptr;
        }
        return *this;
    }

    ~Person()
    {
        // cout << "destructor" << endl;
        delete ptr;
    }

    void info()
    {
        if (ptr) cout << *ptr << endl;
        else cout << "ptr is nullptr" << endl;
    }
};

Person func(Person& p) {
    return p;
}

int main(int argc, char const *argv[])
{
    Person p1("kavin"), p2;
    // Person p2 = std::move(p1);
    // p1.info();
    p2 = func(p1);
    p1.info();
    return 0;
}
