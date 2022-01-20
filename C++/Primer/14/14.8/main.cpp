#include <iostream>
#include <vector>
#include <algorithm>

using namespace std;

class PrintString
{
private:
    ostream &os;
    char sep;
public:
    PrintString(ostream& o = cout, char c = ' '): os(o), sep(c) {}
    void operator()(const string& s) const { os << s << sep; }
};

struct absInt {
    int operator()(int val) const {
        return val < 0 ? -val : val;
    }
};

class ShorterString {
public:
    bool operator()(const string& s1, const string& s2) const {
        return s1.size() < s2.size();
    }
};

class SizeComp {
public:
    SizeComp(size_t n): sz(n) {}
    bool operator()(const string& s) const {
        return s.size() >= sz;
    }
private:
    size_t sz;
};

int main(int argc, char const *argv[])
{
    // int i = -42;
    // absInt absObj;
    // int ui = absObj(i);
    // cout << ui << endl;
    // PrintString printer;
    // string s = "hello";
    // printer(s);

    // vector<string> vs = {"kavin", "jack", "lisa"};
    // for_each(vs.begin(), vs.end(), PrintString(cout, '#'));

    // vector<string> words = {"joe", "jack", "lisa", "kavin"};
    // stable_sort(words.begin(), words.end(), 
    //             [](const string& a, const string& b)
    //             { return a.size() < b.size(); });
    // stable_sort(words.begin(), words.end(), ShorterString());
    
    // sort(words.begin(), words.end(), greater<string>());
    // for (auto w : words) cout << w << endl;
    
    // int sz = 5;
    // auto wc = find_if(words.begin(), words.end(), 
    //                 [sz](const string& a)
    //                 { return a.size() >= sz; });
    // auto wc = find_if(words.begin(), words.end(), SizeComp(sz));
    // cout << *wc << endl;


    string s1 = "kavin";
    string s2 = "jack";
    string s3 = "lisa";

    vector<string*> nameTable = {&s1, &s2, &s3};

    sort(nameTable.begin(), nameTable.end(), greater<string*>());

    for (auto& n : nameTable) cout << *n << endl;

    return 0;
}
