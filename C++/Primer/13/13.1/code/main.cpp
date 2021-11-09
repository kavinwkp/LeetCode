#include <iostream>

using namespace std;

class Person
{
private:
    string name;
    int age;
public:
    Person(string name, int age) : name(name), age(age)
    {
        cout << "constructor" << endl;
    }
    Person(const Person& rhs)
    {
        cout << "copy constructor" << endl;
        name = rhs.name;
        age = rhs.age;
    }
    // Person& operator=(const Person& rhs) {
    //     cout << "-----copy-assignment-----" << endl;
    //     name = rhs.name;
    //     age = rhs.age;
    //     return *this;
    // }
};

void func(Person p) {
    cout << "Entering func" << endl;
    return;
}

Person func2() {
    Person *p = new Person("lisa", 34);
    return *p;
}

int main(int argc, char const *argv[])
{
    // Person p1("kavin", 23);
    // Person p2(p1);
    // Person p3 = p1;
    // func(p1);
    // Person *ptr = new Person("kavin", 34);
    Person p2 = func2();
    return 0;
}

