#ifndef SIMPLE_H
#define SIMPLE_H

#include <iostream>

class simple {
public:
    simple(void) : _x(0), _y(0) {};
    simple(int x, int y) : _x(x), _y(y) {};
    void disp();
    void add();
private:
    int _x, _y;
};



#endif