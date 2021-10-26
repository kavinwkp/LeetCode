#include <iostream>
using namespace std;

struct ListNode
{
    int val;
    ListNode* next;
    ListNode(int x) : val(x), next(nullptr) {}
    ListNode(int x, ListNode* ne) : val(x), next(ne) {}
};

void disp(ListNode *head) {
    if (!head) return;
    cout << head->val << " ";
    disp(head->next);
}

ListNode *rightNode = nullptr;
ListNode* traversal(ListNode *head, int cnt) {
    if (cnt == 0) {
        rightNode = head->next;
        return head;
    }
    ListNode *newHead = traversal(head->next, cnt-1);
    head->next->next = head;
    head->next = nullptr;
    return newHead;
}

int main(int argc, char const *argv[])
{
    ListNode* node5 = new ListNode(5, nullptr);
    ListNode* node4 = new ListNode(4, node5);
    ListNode* node3 = new ListNode(3, node4);
    ListNode* node2 = new ListNode(2, node3);
    ListNode* node1 = new ListNode(1, node2);
    ListNode* head = node1;
    disp(head);
    cout << endl;

    int l = 2;
    int r = 4;
    int len = r - l;
    ListNode *dummy = new ListNode(0, head);
    head = dummy;
    while (--l) head = head->next;
    ListNode *leftNode = head->next;
    head->next = traversal(head->next, len);
    leftNode->next = rightNode;
    disp(dummy->next);
    cout << endl;
    return 0;
}
