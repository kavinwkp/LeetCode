#include <iostream>
#include <vector>
#include <queue>
#include <unordered_set>
#include <algorithm>
#include <random>
#include <ctime>
using namespace std;

struct ListNode {
    int val;
    ListNode *next;
    ListNode() : val(0), next(nullptr) {}
    ListNode(int x) : val(x), next(nullptr) {}
    ListNode(int x, ListNode *ne) : val(x), next(ne) {}
};

void dispListNode(ListNode *head) {
    while (head) {
        cout << head->val << " ";
        head = head->next;
    }
}

void revDispListNode(ListNode *head) {
    if (!head) return;
    revDispListNode(head->next);
    cout << head->val << " ";
}

// struct TreeNode
// {
//     int val;
//     TreeNode *left;
//     TreeNode *right;
//     TreeNode(int x) : val(x), left(nullptr), right(nullptr) {}
//     TreeNode(int x, TreeNode *l, TreeNode *r) : val(x), left(l), right(r) {}
// };

// void disp(TreeNode *root) {
//     if (!root) return;
//     cout << root->val << " ";
//     disp(root->left);
//     disp(root->right);
// }

// int add = 1;
// void addOne(TreeNode *root) {
//     if (!root) return;
//     if (root->left && !root->right) addOne(root->left);
//     else if (!root->left && root->right) addOne(root->right);
//     else if (root->left && root->right) {
//         addOne(root->left);
//         add--;
//         addOne(root->right);
//     }
//     root->val += add;
//     add++;
// }

ListNode* getListNode(vector<int>& nums) {
    ListNode *head = new ListNode(-1);
    ListNode *dummy = head;
    for (auto& n : nums) {
        ListNode *node = new ListNode(n);
        head->next = node;
        head = head->next;
    }
    return dummy->next;
}

int main(int argc, char const *argv[])
{
    // vector<int> nums = {1, 2, 3, 4};
    // ListNode *head = getListNode(nums);
    // dispListNode(head);
    // revDispListNode(head);
    // cout << endl;
    // pair<int, int> q1(1, 2);
    // pair<int, int> q2(10, 2);
    // pair<int, int> q3(5, 0);
    // priority_queue<pair<int,int>, vector<pair<int,int>>, greater<pair<int,int>>> q;
    // q.push(q1);
    // q.push(q2);
    // q.push(q3);
    // cout << q.top().first << endl;
    // q.pop();
    // cout << q.top().first << endl;
    // q.pop();
    // cout << q.top().first << endl;

    // vector<int> nums = {2, 2, 2, 2, 3, 3, 3};
    // int res = 0;
    // int cnt = 1;
    // for (uint32_t i = 1; i < nums.size(); i++) {
    //     if (nums[i] == nums[i - 1]) cnt++;
    //     else cnt = 1;
    //     res = max(res, cnt);
    // }
    // cout << res << endl;
    // int a = -2;
    // int b = a % 10;
    // cout << b << endl;
    // unordered_set<int> hash;
    // hash.insert(10);
    // auto it = hash.find(10);
    // cout << sizeof(it) << endl;
    // cout << sizeof(short) << endl;
    vector<int> test = {1, 20, 3, 9, 100};
    test.insert(test.begin() + 4, -1);
    // cout << *max_element(test.begin(), test.end()) << endl;
    // mt19937 rnd((unsigned)time(0));
    // for (int i = 0; i < 10; i++) {
    //     uniform_int_distribution<> dist(0, test.size() - 1);
    //     cout << dist(rnd) << endl;
    // }
        
    // shuffle(test.begin(), test.end(), rnd);
    for (auto t : test) 
        cout << t << " ";
    cout << endl;

    // float num = 0.0000002;
    // cout << (fabs(num - 0.0) > 1e-6) << endl;
    return 0;
}