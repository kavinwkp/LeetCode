#include <iostream>
#include <vector>
using namespace std;

void merge(vector<int>& nums, int l, int mid, int r) {
    vector<int> res(r - l + 1);
    int cnt = 0;
    int p1 = l, p2 = mid + 1;
    while (p1 <= mid && p2 <= r) {
        if (nums[p1] < nums[p2]) 
            res[cnt++] = nums[p1++];
        else
            res[cnt++] = nums[p2++];
    }
    while (p1 <= mid) res[cnt++] = nums[p1++];
    while (p2 <= l) res[cnt++] = nums[p2++];
    for (uint32_t i = 0; i < res.size(); i++) {
        nums[i + l] = res[i];
    }
}

void mergeSort(vector<int>& nums, int l, int r) {
    if (left >= right) return;
    int mid = (l + r) >> 1;
    mergeSort(nums, l, mid);
    mergeSort(nums, mid + 1, r);
    merge(nums, l, mid, r);
}

int main(int argc, char const *argv[])
{
    vector<int> nums = {4, 1, 3, 2};
    mergeSort(nums, 0, nums.size() - 1);
    for (auto n : nums) {
        cout << n << " ";
    }
    cout << endl;
    return 0;
}