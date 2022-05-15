#include <iostream>
#include <cstring>
#include <memory>
#include <vector>

using namespace std;

class StrBlob {
public:
    typedef vector<string>::size_type size_type;
    StrBlob(): data(make_shared<vector<string>>()) {}
    StrBlob(initializer_list<string> il): data(make_shared<vector<string>>(il)) {}
    size_type size() const { return data->size(); }
    bool empty() const { return data->empty(); }
    void push_back(const string& t) { data->push_back(t); }
    void pop_back();
    string& front();
    string& back();
private:
    shared_ptr<vector<string>> data;
    void check(size_type i, const string& msg) const;
};

void StrBlob::check(size_type i, const string& msg) const {
    if (i >= data->size()) 
        throw out_of_range(msg);
}

string& StrBlob::front() {
    check(0, "front on empty StrBlob");
    return data->front();
}

string& StrBlob::back() {
    check(0, "back on empty StrBlob");
    return data->back();
}

void StrBlob::pop_back() {
    check(0, "pop_back on empty StrBlob");
    data->pop_back();
}

int main() {

    // shared_ptr<int> p3 = make_shared<int>(42);
    // cout << *p3 << endl;
    // shared_ptr<string> p4 = make_shared<string>(10, '9');
    // cout << *p4 << endl;
    // auto p6 = make_shared<vector<string>>();
    // cout << p6->capacity() << endl;
    // auto p = make_shared<int>(42);
    // auto q(p);
    // cout << p.use_count() << endl;

    StrBlob s1{"kavin", "jack", "lisa"};
    cout << s1.size() << endl;
    cout << s1.front() << endl;
    cout << s1.back() << endl;
    s1.pop_back();
    cout << s1.back() << endl;
    return 0;
}