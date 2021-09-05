#include <iostream>

using namespace std;

struct ListNode
{
    int val;
    ListNode* next;
    ListNode(int x) : val(x), next(nullptr) {}
    ListNode(int x, ListNode* ne) : val(x), next(ne) {}
};

void disp_rev(ListNode* cur) {
    if (!cur) return;
    disp_rev(cur->next);
    cout << cur->val << " ";
}

ListNode *front = nullptr;

bool traversal(ListNode* cur) {
    if (!cur) return true;
    if (!traversal(cur->next)) return false;
    if (cur->val != front->val) return false;
    front = front->next;
    return true;
}
int main(int argc, char const *argv[])
{
    ListNode* node3 = new ListNode(1, nullptr);
    ListNode* node2 = new ListNode(2, node3);
    ListNode* node1 = new ListNode(1, node2);
    ListNode* head = node1;
    disp_rev(head);
    cout << endl;
    ListNode *fast = head;
    ListNode *slow = head;
    while (fast && fast->next) {
        fast = fast->next->next;
        slow = slow->next;
    }
    front = head;
    cout << traversal(head);
    return 0;
}
