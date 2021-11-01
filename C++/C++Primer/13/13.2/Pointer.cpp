#include <iostream>

using namespace std;

class Person
{
private:
    string *ptr;
    int age;
    size_t *use;

public:
    Person(const string &s = string()) : ptr(new string(s)), age(0), use(new size_t(1)) {}
    Person(const Person &p) : ptr(p.ptr), age(p.age), use(p.use) { *use++; }
    Person& operator=(const Person& rhs) {
        ++*rhs.use;     // 先递增左侧对象的引用计数
        if (--*use == 0) {  // 再递减右侧对象的引用计数
            delete ptr;
            delete use;
        }
        ptr = rhs.ptr;
        age = rhs.age;
        use = rhs.use;
        return *this;
    }
    ~Person() {
        if (--*use == 0) {  // 引用计数为0就释放资源
            delete ptr;
            delete use;
        }
    }
};

int main(int argc, char const *argv[])
{
    Person p1("kavin");
    Person p2;
    p2 = p1;
    return 0;
}
