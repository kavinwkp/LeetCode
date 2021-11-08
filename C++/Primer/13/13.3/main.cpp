#include <iostream>

using namespace std;

class Person
{
private:
    string *ptr;
    int age;

public:
    Person(const string &s = string(), const int n = 0) : ptr(new string(s)), age(n) {}
    Person(const Person &p) : ptr(new string(*p.ptr)), age(p.age)
    {
        cout << "copy constructor" << endl;
    }
    // Person& operator=(const Person& rhs) {
    //     cout << "copy assignment" << endl;
    //     auto newptr = new string(*rhs.ptr);
    //     delete ptr;     // 释放当前对象的资源
    //     ptr = newptr;   // 指针指向新的资源
    //     age = rhs.age;
    //     return *this;
    // }
    Person &operator=(Person rhs)
    {
        cout << "swap copy assignment" << endl;
        swap(*this, rhs);
        return *this;
    }
    ~Person() { delete ptr; }
    friend void swap(Person &, Person &);
    void info();
};

inline void swap(Person &lhs, Person &rhs)
{
    cout << "Person swap" << endl;
    using std::swap;
    swap(lhs.ptr, rhs.ptr);
    swap(lhs.age, rhs.age);
}

void Person::info()
{
    cout << *ptr << ": " << age << endl;
}

int main(int argc, char const *argv[])
{
    Person p1("kavin", 23);
    Person p2("jack", 34);
    // Person tmp = p1;
    // p1 = p2;
    // p2 = tmp;
    p2 = p1;
    // swap(p1, p2);
    // p1.info();
    // p2.info();
    return 0;
}
