#include <iostream>
using namespace std;

class Point2D {
public:
    Point2D(int _x, int _y): x(_x), y(_y) {}
    virtual void print() const {
        cout << x << "," << y << endl;
    }
    virtual ~Point2D() {}
protected:
    int x, y;
};

class Point3D: public Point2D {
public:
    Point3D(int _x, int _y, int _z): Point2D(_x, _y), z(_z) {}
    void print() const {
        cout << x << "," << y << "," << z << endl;
    }
private:
    int z;
};

int main() {
    Point2D *p2d = new Point3D(1, 2, 3);
    p2d->print();
    Point2D *p2d2 = new Point2D(1, 2);
    p2d2->print();
    return 0;
}
