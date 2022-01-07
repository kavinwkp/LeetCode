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

void revDispListNode(ListNode *head) {
    if (!head) return;
    revDispListNode(head->next);
    cout << head->val << " ";
}

ListNode* getListNode(const vector<int>& nums) {
    ListNode *head = new ListNode(-1);
    ListNode *dummy = head;
    for (auto& n : nums) {
        ListNode *node = new ListNode(n);
        head->next = node;
        head = head->next;
    }
    return dummy->next;
}

ListNode* reverse1(ListNode *head) {
    if (!head || !head->next) return head;
    ListNode *node = reverse1(head->next);
    head->next->next = head;
    head->next = nullptr;
    return node;
}

ListNode* reverse2(ListNode *head) {
    if (!head) return head;
    ListNode *pre = nullptr;
    while (head) {
        ListNode *tmp = head->next;
        head->next = pre;
        pre = head;
        head = tmp;
    }
    return pre;
}

int main(int argc, char const *argv[])
{
    vector<int> nums = {1, 2, 3, 4};
    ListNode *head = getListNode(nums);
    dispListNode(head);
    cout << endl;
    ListNode *revHead = reverse2(head);
    dispListNode(revHead);
    cout << endl;
    // revDispListNode(head);
    // cout << endl;
    return 0;
}