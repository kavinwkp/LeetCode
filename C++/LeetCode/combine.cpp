#include <iostream>
#include <vector>
using namespace std;


vector<int> path;
vector<vector<int>> res;
void combine(int n, int k, int index) {
    if (path.size() == k) {
        res.push_back(path);
        return;
    }
    for (int i = index; i <= n - (k - path.size()) + 1; i++) {
        path.push_back(i);
        combine(n, k, i + 1);
        path.pop_back();
    }
}

int main(int argc, char const *argv[])
{
    int n = 4;
    int k = 3;
    combine(n, k, 1);
    for (int i = 0; i < res.size(); i++) {
        for (int j = 0; j < res[0].size(); j++) {
            cout << res[i][j] << " ";
        }
        cout << endl;
    }
    return 0;
}