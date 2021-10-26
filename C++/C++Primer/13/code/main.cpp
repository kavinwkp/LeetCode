#include <iostream>

using namespace std;

class Person
{
private:
    string name;
    int age;
public:
    Person(string name, int age) : name(name), age(age) {}
    Person(const Person& rhs) {
        cout << "-----copy constructor-----" << endl;
        name = rhs.name;
        age = rhs.age;
    }
    Person& operator=(const Person& rhs) {
        cout << "-----copy-assignment-----" << endl;
        name = rhs.name;
        age = rhs.age;
        return *this;
    }
};

int main(int argc, char const *argv[])
{
    Person p1("kavin", 23);
    Person p2(p1);
    // Person p3("jack", 12);
    // p3 = p1;
    return 0;
}

