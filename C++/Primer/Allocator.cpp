#include <iostream>

using std::cout;
using std::endl;

class allocator {
private:
    struct obj {
        struct obj* next;
    };
public:
    void* allocate(size_t);
    void deallocate(void*, size_t);
private:
    obj* freeStore = nullptr;
    const int CHUNK = 5;
};

void* allocator::allocate(size_t size) {
    obj* p;
    if (!freeStore) {
        size_t chunk = CHUNK * size;
        freeStore = p = (obj*)malloc(chunk);
        for (int i = 0; i < (CHUNK - 1); ++i) {
            p->next = (obj*)((char*)p + size);
            p = p->next;
        }
        p->next = nullptr;
    }
    p = freeStore;
    freeStore = freeStore->next;
    return p;
}

void allocator::deallocate(void* p, size_t) {
    ((obj*)p)->next = freeStore;
    freeStore = (obj*)p;
}

class Foo {
public:
    long a;
    double b;
    static allocator myAlloc;
public:
    Foo(long x): a(x) {}
    static void* operator new(size_t size) {
        return myAlloc.allocate(size);
    }
    static void operator delete(void* pdead, size_t size) {
        return myAlloc.deallocate(pdead, size);
    }
};

allocator Foo::myAlloc;

// class Foo {
// public:
//     long a;
//     double b;
// public:
//     Foo(long x): a(x) {}
// };

int main() {
    Foo *p[100];
    cout << sizeof(Foo) << endl;
    for (int i = 0; i < 10; i++) {
        p[i] = new Foo(i);
        cout << p[i] << ' ' << p[i]->a << endl;
    }
    for (int i = 0; i < 10; i++) {
        delete p[i];
    }
    return 0;
}

// 16
// 0x559b42e3b280 0
// 0x559b42e3b290 1
// 0x559b42e3b2a0 2
// 0x559b42e3b2b0 3
// 0x559b42e3b2c0 4
// 0x559b42e3b2e0 5
// 0x559b42e3b2f0 6
// 0x559b42e3b300 7
// 0x559b42e3b310 8
// 0x559b42e3b320 9

// 16
// 0x5633e46ad280 0
// 0x5633e46ad2a0 1
// 0x5633e46ad2c0 2
// 0x5633e46ad2e0 3
// 0x5633e46ad300 4
// 0x5633e46ad320 5
// 0x5633e46ad340 6
// 0x5633e46ad360 7
// 0x5633e46ad380 8
// 0x5633e46ad3a0 9