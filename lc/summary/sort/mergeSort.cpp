#include <iostream>
#include <vector>
using namespace std;

void merge(vector<int>& nums, int l, int mid, int r, vector<int>& tmp) {
    int cnt = l;
    int i = l, j = mid + 1;
    while (i <= mid && j <= r) {
        if (nums[i] <= nums[j]) tmp[cnt++] = nums[i++];
        else tmp[cnt++] = nums[j++];
    }
    while (i <= mid) tmp[cnt++] = nums[i++];
    while (j <= r) tmp[cnt++] = nums[j++];
    for (int k = l; k <= r; k++) {
        nums[k] = tmp[k];
    }
}

void mergeSort(vector<int>& nums, int l, int r, vector<int>& tmp) {
    if (l >= r) return;
    int mid = l + (r - l) / 2;
    mergeSort(nums, l, mid, tmp);
    mergeSort(nums, mid + 1, r, tmp);
    merge(nums, l, mid, r, tmp);
}

int main(int argc, char const *argv[])
{
    vector<int> nums = {3, 2, 1, 4, 5};
    int n = nums.size();
    vector<int> tmp(n);
    mergeSort(nums, 0, n - 1, tmp);
    for (auto& n : nums) cout << n << ' ';
    cout << endl;
    return 0;
}