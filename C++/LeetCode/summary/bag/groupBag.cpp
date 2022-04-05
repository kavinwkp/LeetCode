#include <iostream>
using namespace std;

const int N = 105;

int dp[N], v[N], w[N];
int n, m;
int main() {
    cin >> n >> m;
    for (int i = 0; i < n; i++) {
        int s;
        cin >> s;
        for (int j = 0; j < s; j++) cin >> v[j] >> w[j];
        for (int j = m; j >= 0; j--) {
            for (int k = 0; k < s; k++) {
                if (j >= v[k]) dp[j] = max(dp[j], dp[j - v[k]] + w[k]);
            }
        }
    }
    cout << dp[m];
    return 0;
}
// Input:
// 3 5
// 2
// 1 2
// 2 4
// 1
// 3 4
// 1
// 4 5
// Output:
// 8