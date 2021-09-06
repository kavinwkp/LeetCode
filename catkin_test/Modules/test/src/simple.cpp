#include "test/simple.h"


void simple::disp() {
    std::cout << "simple: " << "(" << _x << ", " << _y << ")" << std::endl;
}

void simple::add() {
    this->_x += 1;
    this->_y += 2;
}