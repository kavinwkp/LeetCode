#include <iostream>
#include <vector>
using namespace std;

string shiftingLetters(string s, vector<int>& nums) {
    for (int i = nums.size() - 2; i >= 0; i--) {
        nums[i] = (nums[i] + nums[i + 1]) % 26;
    }
    nums.back() %= 26;
    // for (auto n : nums)
    //     cout << n << " ";
    // cout << endl;
    for (int i = 0; i < s.size(); i++) {
        s[i] = (s[i] - 'a' + nums[i]) % 26 + 'a';
    }
    return s;
}

int main(int argc, char const *argv[])
{
    string s = "ruu";
    vector<int> nums = {26,9,17};
    cout << shiftingLetters(s, nums) << endl;
    return 0;
}
