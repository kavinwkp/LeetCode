#include <iostream>
#include <vector>

using namespace std;

class Screen
{
public:
    typedef string::size_type pos;
    Screen() = default;
    Screen(pos h, pos w, char c)
        : height(h), width(w), contents(h * w, c) {}
    char get() const { return contents[cursor]; }   // 隐式内联
    inline char get(pos r, pos c) const;    // 显式内联
    Screen& move(pos r, pos c);     // 能在之后被设为内联
    Screen& set(char);
    Screen& set(pos, pos, char);
    Screen& display(ostream& os) {
        do_display(os);
        return *this;
    }
    const Screen& display(ostream& os) const {
        do_display(os);
        return *this;
    }

private:
    pos cursor = 0;
    pos height = 0, width = 0;
    string contents;
    void do_display(ostream& os) const { os << contents; }

    // friend class Window_mgr;
    friend void Window_mgr::clear(ScreenIndex);
};

char Screen::get(pos r, pos c) const {
    pos row = r * width;
    return contents[row + c];   // 获取指定位置的字符
}

inline  // 可以在函数的定义处指定inline
Screen& Screen::move(pos r, pos c) {
    pos row = r * width;
    cursor = row + c;   // 移动光标到指定位置
    return *this;
}

inline Screen& Screen::set(char c) {
    contents[cursor] = c;
    return *this;
}

inline Screen& Screen::set(pos r, pos c, char ch) {
    contents[r * width + c] = ch;
    return *this;
}

class Window_mgr {
public:
    using ScreenIndex = vector<Screen>::size_type;
    void clear(ScreenIndex);
private:
    vector<Screen> screens{Screen(24, 80, '*')};
};

void Window_mgr::clear(ScreenIndex i) {
    Screen& s = screens[i];     // 指向要清空的那个Screen
    s.contents = string(s.height * s.width, ' ');
}

int main(int argc, char const *argv[])
{
    Screen myScreen(3, 3, 'X');
    myScreen.move(1, 1).set('#').display(cout);
    cout << endl;
    myScreen.display(cout);
    cout << endl;
    return 0;
}
