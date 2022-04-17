#include <iostream>
#include <cstring>
#include <vector>
#include <memory>
using namespace std;

template<typename T>
class Blob {
public:
    typedef T value_type;
    typedef typename vector<T>::size_type size_type;
    Blob();
    Blob(initializer_list<T> il);
    // Blob中的元素数量 
    size_type size() const { return data->size(); }
    // 添加和删除元素
    void push_back(const T& t) { data->push_back(t); }
    // 移动版本
    void push_back(T&& t) { data->push_back(std::move(t)); }
    void pop_back();
    // 元素访问
    T& back();
    T& operator[](size_type i);
private:
    shared_ptr<vector<T>> data;
    void check(size_type i, const string& msg) const;
};

template<typename T>
Blob<T>::Blob(): data(make_shared<vector<T>>()) {}

template<typename T>
Blob<T>::Blob(initializer_list<T> il): data(make_shared<vector<T>>(il)) {}

template<typename T>
void Blob<T>::check(size_type i, const string& msg) const {
    if (i >= data->size()) {
        throw std::out_of_range(msg);
    }
}

template<typename T>
T& Blob<T>::back() {
    check(0, "back on empty Blob"); // 没有元素报错
    return data->back();
}

template<typename T>
T& Blob<T>::operator[](size_type i) {
    check(i, "subscript out of range");
    return (*data)[i];
}

template<typename T>
void Blob<T>::pop_back() {
    check(0, "pop_back on empty Blob");
    data->pop_back();
}

int main(int argc, char const *argv[])
{
    Blob<int> ia;
    cout << ia.size() << endl;
    Blob<int> ia2 = {0, 1, 2, 3, 4};
    cout << ia2.size() << endl;
    ia2.pop_back();
    cout << ia2.size() << endl;
    cout << ia2.back() << endl;
    Blob<string> articles = {"a", "an", "the"};
    cout << articles.size() << endl;
    return 0;
}
