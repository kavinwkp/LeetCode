#include <iostream>

using namespace std;

class Shape {
public:
    virtual void draw(int i = -1) const = 0;
};

class Rectangle : public Shape {
public:
    virtual void draw(int i = 1) const {
        cout << "Rectangle: " << i << endl;
    }
};

class Circle : public Shape {
public:
    virtual void draw(int i) const {
        cout << "Circle: " << i << endl;
    }
};

int main(int argc, char const *argv[])
{
    Shape *pr = new Rectangle;
    Shape *pc = new Circle;
    pr->draw(3);    // Rectangle: 3
    pc->draw(4);    // Circle: 4

    pr->draw(); // Rectangle: -1
    pc->draw(); // Circle: -1

    Circle x;
    x.draw();   // 静态绑定下，无法从base拿到默认值

    return 0;
}