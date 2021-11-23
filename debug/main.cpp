#include <iostream>

using namespace std;

struct ListNode {
    int val;
    ListNode *next;
    ListNode() : val(0), next(nullptr) {}
    ListNode(int x) : val(x), next(nullptr) {}
    ListNode(int x, ListNode *next) : val(x), next(next) {}
};

ListNode *tmp = nullptr;
ListNode* traversal(ListNode *head, int left, int right, int depth) {
    if (depth >= right) {
        tmp = head->next;
        return head;
    }
    ListNode *node = traversal(head->next, left, right, depth + 1);
    if (depth >= left) {
        head->next->next = head;
        head->next = nullptr;
    }
    else if (depth == left - 1) {
        head->next->next = tmp;
        head->next = node;
    }
    return node;
}
ListNode* reverseBetween(ListNode* head, int left, int right) {
    ListNode *dummy = new ListNode(0, head);
    traversal(dummy, left, right, 0);
    return dummy->next;
}
void disp(ListNode *head) {
    if (!head) return;
    cout << head->val << "->";
    disp(head->next);
}

int main(int argc, char const *argv[])
{
    // ListNode *node5 = new ListNode(5, nullptr);
    // ListNode *node4 = new ListNode(4, node5);
    ListNode *node3 = new ListNode(3, nullptr);
    ListNode *node2 = new ListNode(2, node3);
    ListNode *node1 = new ListNode(1, node2);
    disp(node1);
    cout << endl;
    node1 = reverseBetween(node1, 1, 2);
    disp(node1);
    cout << endl;
    return 0;
}
