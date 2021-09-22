#include <iostream>
#include <vector>
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

// int add = 1;
// void addOne(ListNode *head) {
//     if (!head) return;
//     addOne(head->next);
//     head->val += add;
//     add++;
// }

// int main() {
//     ListNode *node3 = new ListNode(3);
//     ListNode *node2 = new ListNode(2, node3);
//     ListNode *node1 = new ListNode(1, node2);
//     ListNode *head = node1;
//     disp(head);
//     cout << endl;
//     addOne(head);
//     disp(head);
//     cout << endl;
//     return 0;
// }

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
    vector<int> nums = {1, 2, 3, 4};
    ListNode *head = getListNode(nums);
    dispListNode(head);
    cout << endl;
    return 0;
}