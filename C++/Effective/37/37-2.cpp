#include <iostream>

using namespace std;

class Shape {
public:
    void draw(int i = -1) const {
        doDraw(i);
    }
private:
    virtual void doDraw(int i) const = 0;
};

class Rectangle : public Shape {
private:
    virtual void doDraw(int i) const {
        cout << "Rectangle: " << i << endl;
    }
};

class Circle : public Shape {
private:
    virtual void doDraw(int i) const {
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
    x.draw();   // Circle: -1

    return 0;
}