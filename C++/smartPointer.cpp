#include <iostream>
#include <memory>

using namespace std;

class Entity
{
public:
    Entity() { cout << "Entity created!" << endl; }
    ~Entity() {cout << "Entity destroyed!" << endl; }
};

void ex1() {
    puts(">>>>>Entering ex1<<<<<");
    {
        puts("-----Entering scope-----");
        auto e1 = std::make_unique<Entity>();
        puts("-----Leaving scope-----");
    }
    puts(">>>>>Leaving ex1<<<<<");
}

void foo(std::unique_ptr<Entity>) {
    puts("-----Entering foo-----");
    puts("-----Leaving foo-----");
}

void ex2() {
    puts(">>>>>Entering ex2<<<<<");
    auto e1 = std::make_unique<Entity>();
    foo(std::move(e1));
    puts(">>>>>Leaving ex2<<<<<");
}

void ex3() {
    puts(">>>>>Entering ex3<<<<<");
    auto e1 = std::make_shared<Entity>();
    std::cout << e1.use_count() << std::endl;
    {
        puts("-----Entering scope-----");
        auto e2 = e1;
        std::cout << e1.use_count() << std::endl;
        auto e3 = std::move(e2);
        std::cout << e1.use_count() << std::endl;
        puts("-----Leaving scope-----");
    }
    std::cout << e1.use_count() << std::endl;
    puts(">>>>>Leaving ex3<<<<<");
}

void observe(std::weak_ptr<Entity> ew) {
    puts("*****Entering observe*****");
    if (std::shared_ptr<Entity> spt = ew.lock()) {
        std::cout << spt.use_count() << std::endl;
        std::cout << "entity still alive" << std::endl;
    } else {
        std::cout << "entity was expired" << std::endl;
    }
    puts("*****Leaving observe*****");
}

void ex4() {
    puts(">>>>>Entering ex4<<<<<");
    std::weak_ptr<Entity> ew;
    {
        puts("-----Entering scope-----");
        auto e1 = std::make_shared<Entity>();
        std::cout << e1.use_count() << std::endl;
        ew = e1;
        std::cout << e1.use_count() << std::endl;
        observe(ew);
        puts("-----Leaving scope-----");
    }
    observe(ew);
    puts(">>>>>Leaving ex4<<<<<");

}

int main(int argc, char const *argv[])
{
    // ex1();
    // ex2();
    // ex3();
    ex4();
    return 0;
}

// >>>>>Entering ex1<<<<<
// -----Entering scope-----
// Entity created!
// -----Leaving scope-----
// Entity destroyed!
// >>>>>Leaving ex1<<<<<


// >>>>>Entering ex2<<<<<
// Entity created!
// -----Entering foo-----
// -----Leaving foo-----
// Entity destroyed!
// >>>>>Leaving ex2<<<<<

// >>>>>Entering ex4<<<<<
// -----Entering scope-----
// Entity created!
// 1
// 1
// *****Entering observe*****
// 2
// entity still alive
// *****Leaving observe*****
// -----Leaving scope-----
// Entity destroyed!
// *****Entering observe*****
// entity was expired
// *****Leaving observe*****
// >>>>>Leaving ex4<<<<<
