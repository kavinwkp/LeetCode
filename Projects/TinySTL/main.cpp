#include <iostream>
using namespace std;

class T {
public:
    T() { cout << "构造函数" << endl; }
    ~T() { cout << "析构函数" << endl; }
    void* operator new(size_t sz) {
        T* t = (T*)malloc(sizeof(T));
        cout << "内存分配" << endl;
        return t;
    }
    void operator delete(void* p) {
        free(p);
        cout << "内存释放" << endl;
    }
};

int main() {
    T* t = new T();
    delete t;
    return 0;
}
// 内存分配
// 构造函数
// 析构函数
// 内存释放