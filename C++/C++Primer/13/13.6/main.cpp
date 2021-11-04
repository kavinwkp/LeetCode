#include <iostream>

using namespace std;

class Data
{
private:
    int num;
public:
    Data() : num(0) {}
    Data(int n) : num(n) {}
    void info() { cout << num << endl; }
    void modify(int input) {
        num = input;
    }
};

void fun(Data& data) {
    cout << "call l-ref" << endl;
}

void fun(Data&& data) {
    cout << "call r-ref" << endl;    
}

void fun(const Data& data) {
    cout << "call c-ref" << endl;
}

int main(int argc, char const *argv[])
{
    Data data;
    // Data& data1 = data;                 // OK
    // Data& data1 = Data{};               // not compile: invalid binding
    // Data&& data2 = Data{};              // OK
    // Data&& data2 = data;                // not compile: invalid binding
    // Data&& data2 = std::move(data);     // OK

    // fun(data);      // 1, data is lvalue
    // fun(Data());    // 2, data is rvalue
    // fun(data1);     // 1, data1 is l-ref type and lvalue
    // fun(data2);     // 1, data2 is r-ref type but lvalue

    // func(data);         // ok, data is lvalue
    // func(Data());       // ok, Data() is lvalue

    // Data&& ref = Data();
    // ref.info();
    // ref.modify(10);
    // ref.info();
    fun(Data());
    return 0;
}