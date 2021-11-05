#include <iostream>
#include <vector>
#include <queue>

using namespace std;

class TreeNode {
public:
    int val;
    TreeNode *left;
    TreeNode *right;
    TreeNode() : val(0), left(nullptr), right(nullptr) {}
    TreeNode(int x) : val(x), left(nullptr), right(nullptr) {}
    TreeNode(int x, TreeNode *left, TreeNode *right) : val(x), left(left), right(right) {}
};

void flatten(TreeNode* root) {
    if (root->left) flatten(root->left);
    if (root->right) flatten(root->right);
    TreeNode *tmp = root->right;
    root->right = root->left;
    root->left = nullptr;
    TreeNode *cur = root->right;
    while (cur->right) cur = cur->right;
    cur->right = tmp;
}

vector<vector<int>> disp(TreeNode *root) {
    vector<vector<int>> res;
    if (!root) return res;
    queue<TreeNode*> q;
    q.push(root);
    while (!q.empty()) {
        int size = q.size();
        vector<int> path;
        for (int i = 0; i < size; i++) {
            TreeNode *cur = q.front();
            q.pop();
            path.push_back(cur->val);
            if (cur->left) q.push(cur->left);
            if (cur->right) q.push(cur->right);
        }
        res.push_back(path);
    }
    return res;
}

int main(int argc, char const *argv[])
{
    TreeNode *node1 = new TreeNode(1);
    TreeNode *node2 = new TreeNode(2);
    TreeNode *node3 = new TreeNode(3);
    node2->left = node3;
    node1->right = node2;
    flatten(node1);
    return 0;
}
