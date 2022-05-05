#include <iostream>
#include <vector>
using namespace std;

void heapify(vector<int>& nums, int i, int n) {
    int l = 2 * i + 1, r = 2 * i + 2;
    int max = i;
    if (l < n && nums[l] > nums[i]) max = l;
    if (r < n && nums[r] > nums[i]) max = r;
    if (max != i) {
        swap(nums[i], nums[max]);
        heapify(nums, max, n);
    } 
}

void buildHeap(vector<int>& nums, int n) {
    for (int i = n / 2; i >= 0; i--) {
        heapify(nums, i, n);
    }
}

void heapSort(vector<int>& nums) {
    int n = nums.size();
    buildHeap(nums, n);
    for (int i = n - 1; i >= 0; i--) {
        swap(nums[0], nums[i]);
        heapify(nums, 0, i);
    }
}

int main(int argc, char const *argv[])
{
    vector<int> nums = {3, 2, 1, 4, 5};
    heapSort(nums);
    for (auto& n : nums) cout << n << ' ';
    cout << endl;
    return 0;
}