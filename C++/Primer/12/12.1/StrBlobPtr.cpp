#include <iostream>
#include <cstring>
#include <memory>
#include <vector>

using namespace std;

class StrBlobPtr;

class StrBlob {
public:
    friend class StrBlobPtr;
    StrBlobPtr begin();
    StrBlobPtr end();
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

class StrBlobPtr {
public:
    friend bool operator==(const StrBlobPtr&, const StrBlobPtr&);
    friend bool operator!=(const StrBlobPtr&, const StrBlobPtr&);
    StrBlobPtr(): curr(0) {}
    StrBlobPtr(StrBlob& a, size_t sz = 0): wptr(a.data),curr(sz) {}
    string& operator[](size_t) const;
    StrBlobPtr& operator++();
private:
    shared_ptr<vector<string>> check(size_t, const string&) const;
    weak_ptr<vector<string>> wptr;
    size_t curr;
};

shared_ptr<vector<string>> 
StrBlobPtr::check(size_t i, const string& msg) const {
    auto ret = wptr.lock();
    if (!ret) {
        throw runtime_error("unbound StrBlobPtr");
    }
    if (i >= ret->size()) {
        throw out_of_range(msg);
    }
    return ret;
}

string& StrBlobPtr::operator[](size_t n) const {
    auto p = check(curr + n, "dereference past end");
    return (*p)[curr + n];
}


StrBlobPtr& StrBlobPtr::operator++() {
    check(curr, "increment past end of StrBlobPtr");
    ++curr;
    return *this;
}

bool operator==(const StrBlobPtr& lhs, const StrBlobPtr& rhs) {
    return lhs.curr == rhs.curr;
}

bool operator!=(const StrBlobPtr& lhs, const StrBlobPtr& rhs) {
    return !(lhs == rhs);
}


StrBlobPtr StrBlob::begin() { 
    return StrBlobPtr(*this);
}
StrBlobPtr StrBlob::end() {
    auto ret = StrBlobPtr(*this, data->size());
    return ret;
}

int main() {
    StrBlob s1{"kavin", "jack", "lisa"};
    auto ptr = s1.begin();
    for (int i = 0; i < s1.size(); i++) {
        cout << ptr[i] << endl;
    }
    return 0;
}