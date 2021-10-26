#include <iostream>
using namespace std;

struct TreeNode
{
    int val;
    TreeNode *left;
    TreeNode *right;
    TreeNode() : val(0), left(nullptr), right(nullptr) {}
    TreeNode(int x) : val(x), left(nullptr), right(nullptr) {}
    TreeNode(int x, TreeNode *l, TreeNode *r) : val(x), left(l), right(r) {}
};

void disp(TreeNode *root) {
    if (!root) return;
    disp(root->left);
    cout << root->val << " ";
    disp(root->right);
}

TreeNode* deleteNode(TreeNode *root, int key) {
    if (!root) return root;
    if (key == root->val) {
        if (!root->left) return root->right;
        else if (!root->right) return root->left;
        else {
            TreeNode *cur = root->right;
            while (cur->left) cur = cur->left;
            cur->left = root->left;
            TreeNode *tmp = root;
            root = root->right;
            delete tmp;
            return root;
        }
    }
    if (key < root->val) root->left = deleteNode(root->left, key);
    else root->right = deleteNode(root->right, key);
    return root;
}

int main(int argc, char const *argv[])
{
    TreeNode *node1 = new TreeNode(1);
    TreeNode *node4 = new TreeNode(4);
    TreeNode *node3 = new TreeNode(3, node1, node4);
    TreeNode *node8 = new TreeNode(8);
    TreeNode *root = new TreeNode(5, node3, node8);
    disp(root);
    cout << endl;
    int key = 3;
    TreeNode *newRoot = deleteNode(root, key);
    disp(newRoot);
    cout << endl;
    return 0;
}