#include <iostream>
#include <vector>
#include <limits.h>
using namespace std;

double findMedianSortedArrays(vector<int>& nums1, vector<int>& nums2) {
    if (nums1.size() > nums2.size()) swap(nums1, nums2);

    int m = nums1.size(), n = nums2.size();
    int totalLeft = (m + n + 1) / 2;
    int l = 0, r = m;
    while (l < r) {
        int i = (l + r + 1) / 2;
        int j = totalLeft - i;
        if (nums1[i - 1] <= nums2[j]) l = i;
        else r = i - 1;
    }
    int i = l;
    int j = totalLeft - i;
    int nums1Left = (i == 0) ? INT_MIN : nums1[i - 1];
    int nums1Right = (i == m) ? INT_MAX : nums1[i];
    int nums2Left = (j == 0) ? INT_MIN : nums2[j - 1];
    int nums2Right = (j == n) ? INT_MAX : nums2[j];
    if ((m + n) % 2) {
        return max(nums1Left, nums2Left);
    }
    else {
        return (max(nums1Left, nums2Left) + min(nums1Right, nums2Right)) / 2.0;
    }
}

void func(int cnt) {
    cout << cnt << endl;
}
int main() {
    // vector<int> nums1{1,3,4};
    // cout << INT_MAX << endl;
    // cout << INT_MIN << endl;
    // int l = 0, r = nums1.size();
    // while (l < r) {
    //     int mid = (l + r + 1) / 2;
    //     if (nums1[mid] <= 4) l = mid;
    //     else r = mid - 1;
    // }
    // cout << l << endl;
    // vector<int> nums2{2};
    // cout << findMedianSortedArrays(nums1, nums2) << endl;
    int a = 10;
    int b = 100;
    func(b - a);
    return 0;
}
