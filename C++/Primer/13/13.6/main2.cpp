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

    Person& operator=(Person rhs) {
        cout << "swap copy assignment" << endl;
        swap(*this, rhs);
        return *this;
    }
    friend void swap(Person&, Person&);
    ~Person() { delete ptr; }
};

inline
void swap(Person& lhs, Person& rhs) {
    using std::swap;
    swap(lhs.ptr, rhs.ptr);
    swap(lhs.age, rhs.age);
}

int main(int argc, char const *argv[])
{
    Person p1("kavin"); // 直接初始化
    // Person p2(p1);      // 调用拷贝构造函数直接初始化
    // Person p2 = p1;     // 拷贝初始化，编译器会优化为上面那种形式
    Person p2;
    p2 = p1;
    cout << "----------" << endl;
    p2 = std::move(p1);
    return 0;
}
