#include <iostream>
#include <vector>
using namespace std;


int main(int argc, char const *argv[])
{
    vector<int> weights = {1, 3, 4};
    vector<int> value = {15, 20, 30};
    int bagweight = 4;

    vector<int> dp(bagweight + 1, 0);

    for (uint i = 0; i < weights.size(); i++) {
        for (int j = bagweight; j >= weights[i]; j--) {
            dp[j] = max(dp[j], dp[j - weights[i]] + value[i]);
        }
        for (auto n : dp)
            cout << n << "\t";
        cout << endl;
    }

    // vector<vector<int>> dp(weights.size(), vector<int>(bagweight + 1, 0));
    // for (int j = bagweight; j >= weights[0]; j--)
    //     dp[0][j] = dp[0][j - weights[0]] + value[0];

    // for (uint32_t i = 1; i < value.size(); i++) {
    //     for (int j = 0; j <= bagweight; j++) {
    //         if (j >= weights[i]) 
    //             dp[i][j] = max(dp[i - 1][j], dp[i - 1][j - weights[i]] + value[i]);
    //     }
    // }
    // cout << dp[weights.size() - 1][bagweight] << endl;
    // for (uint i = 0; i < dp.size(); i++) {
    //     for (int j = 0; j <= bagweight; j++) {
    //         cout << dp[i][j] << "\t";
    //     }
    //     cout << endl;
    // }

    return 0;
}