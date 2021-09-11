#include <iostream>
#include <cmath>
#include <vector>
using namespace std;

int main() {
    vector<float> time;
    for (float i = 0; i < 2; i += 0.1) {
        time.push_back(i);
    }
    // for (uint32_t i = 0; i < time.size(); i++) {
    //    cout << time[i] << " ";
    // }
    // cout << endl;
    vector<float> value;
    for (uint32_t i = 0; i < time.size(); i++) {
        float v = 2 * sin(M_PI * time[i]);
        cout << v << endl;
        value.push_back(v);
    }
    return 0;
}
