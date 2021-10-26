#include <iostream>
using namespace std;

int count[5];

int maxNumberOfBalloons(string text) {
    for (auto s : text) {
        if (s == 'a') count[0]++;
        else if (s == 'b') count[1]++;
        else if (s == 'l') count[2]++;
        else if (s == 'n') count[3]++;
        else if (s == 'o') count[4]++;
    }
    int num = 0;
    for (int i = 0; i < 5; i++) {
        if (i == 2 || i == 4) num = min(num, count[i] / 2);
        else num = min(num, count[i]);
    }
    return num;
}

int main() {
    // string s = "lloo";
    // cout << maxNumberOfBalloons(s);
    return 0;
}
