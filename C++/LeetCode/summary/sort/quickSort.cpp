#include <iostream>
#include <vector>
using namespace std;

void quickSort(vector<int>& nums, int l, int r) {
    if (l >= r) return;
    int pivot = nums[r];
    int i = l;
    for (int j = l; j <= r; j++) {
        if (nums[j] < pivot) {
            swap(nums[j], nums[i]);
            i++;
        }
    }
    swap(nums[i], nums[r]);
    quickSort(nums, l, i - 1);
    quickSort(nums, i + 1, r);
}

int main(int argc, char const *argv[])
{
    vector<int> nums = {3, 2, 1, 4, 5};
    quickSort(nums, 0, nums.size() - 1);
    for (auto& n : nums) cout << n << ' ';
    cout << endl;
    return 0;
}