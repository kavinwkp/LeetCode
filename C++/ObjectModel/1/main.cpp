#include <iostream>
using namespace std;

struct Point {
public:
    void print() {
        cout << "&m_x=" << &m_x << endl;
        cout << "&m_y=" << &m_y << endl;
        cout << "&m_z=" << &m_z << endl;
    }
private:
    int m_x;
public:
    int m_y;
    int m_z;
};

int main(int argc, char const *argv[])
{
    Point pt;
    pt.print();
    return 0;
}
