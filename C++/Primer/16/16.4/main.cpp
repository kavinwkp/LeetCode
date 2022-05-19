#include <iostream>

using namespace std;

template<typename T, typename... Args>
void foo(const T& t, const Args&... rest) {
    cout << sizeof...(Args) << endl;
    cout << sizeof...(rest) << endl;
}

template<typename T>
ostream& print(ostream& os, const T& t) {
    return os << t;
}

template<typename T, typename... Args>
ostream& print(ostream& os, const T& t, const Args... rest) {
    os << t << ", ";
    return print(os, rest...);
}

int main() {
    int i = 0;
    double d = 3.14;
    string s =  "hello";
    // foo(i, s, 42, d);
    // foo(s, 42, "hi");
    // foo(d, s);
    // foo("hi");
    print(cout, i, s, 42);
    return 0;
}