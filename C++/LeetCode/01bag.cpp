#include <iostream>
#include <vector>
#include <algorithm>
using namespace std;

int main(int argc, char const *argv[])
{
    vector<int> weight = {1, 3, 4};
    vector<int> value = {15, 20, 30};

    int bagWeight = 4;
    vector<int> dp(bagWeight + 1, 0);

    for (int i = 0; i < weight.size(); i++) {
        for (int j = bagWeight; j >= weight[i]; j--) {
            dp[j] = max(dp[j], dp[j - weight[i]] + value[i]);
        }
        for (int k = 0; k <= bagWeight; k++) 
            cout << dp[k] << " ";
        cout << endl;
    }
    return 0;
}