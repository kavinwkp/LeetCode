#include <iostream>
#include <map>
#include <functional>

using namespace std;

// 普通函数
int add(int i, int j) { return i + j;}
// lambda，其产生一个未命名的函数对象类
auto mod = [](int i, int j) { return i % j; };
// 函数对象类
struct divide {
    int operator()(int denominator, int devisor) {
        return denominator / devisor;
    }
};

int main(int argc, char const *argv[])
{
    // map<string, int(*)(int, int)> binops;
    // binops.insert({"+", add});
    // binops.insert({"%", mod});

    // function<int(int, int)> f1 = add;
    // function<int(int, int)> f2 = divide();
    // function<int(int, int)> f3 = [](int i, int j)
    //                             { return i % j; };
    // cout << f1(4, 2) << endl;
    // cout << f2(4, 2) << endl;
    // cout << f3(4, 2) << endl;

    map<string, function<int(int, int)>> binops = {
        {"+", add},             // 函数指针
        {"-", minus<int>()},    // 标准库函数对象
        {"/", divide()},        // 用户定义的函数对象
        {"*", [](int i, int j){return i * j;}}, // 未命名的lambda
        {"%", mod}              // 命名的lambda
    };

    cout << binops["+"](10, 5) << endl;
    cout << binops["-"](10, 5) << endl;
    cout << binops["/"](10, 5) << endl;
    cout << binops["*"](10, 5) << endl;
    cout << binops["%"](10, 5) << endl;

    return 0;
}
