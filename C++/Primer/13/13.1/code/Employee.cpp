#include <iostream>

using namespace std;

class Employee
{
private:
    string name;
    static int id = 100;
public:
    Employee();
    Employee(string name) : name(name) {
        id = id + 1;
    }
    ~Employee();
};


int main(int argc, char const *argv[])
{
    Employee s1("kavin");
    return 0;
}


