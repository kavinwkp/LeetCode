#include <iostream>
#include "test.h"
using namespace std;

#define MAXLEVEL 3

int get_random_level() {
    int k = 1;
    while (rand() % 2) {
        k++;
    }
    k = (k < MAXLEVEL) ? k : MAXLEVEL;
    return k;
}

int list_level;
int elem_count;

Node *head = new Node(0, 0, MAXLEVEL);

void display() {
    for (int i = 0; i <= head->node_level; i++) {
        Node *cur = head->forward[i];
        cout << i << " level: ";
        while (cur) {
            printf("[%d:%d] ", cur->get_key(), cur->get_value());
            cur = cur->forward[i];
        }
        cout << endl;
    }
}

bool search_node(int key) {
    Node *cur = head;
    for (int i = list_level; i >= 0; i--) { // 从当前跳表的最大层开始找
        while (cur->forward[i] && cur->forward[i]->get_key() < key) {
            cur = cur->forward[i];
        }
    }
    cur = cur->forward[0];
    if (cur && cur->get_key() == key) {
        cout << "Found key " << key << ":" << cur->get_value() << endl;
        return true;
    }
    cout << "Not Found key " << key << endl;
    return false;
}

int insert_node(int key, int value) {
    Node *cur = head;
    Node *update[MAXLEVEL + 1];
    memset(update, 0, sizeof(Node*) * (MAXLEVEL + 1));
    for (int i = list_level; i >= 0; i--) {
        while (cur->forward[i] && cur->forward[i]->get_key() < key) {
            cur = cur->forward[i];
        }
        update[i] = cur;
    }
    cur = cur->forward[0];
    if (cur != NULL && cur->get_key() == key) {
        cout << "key: " << key << ", exits" << endl;
        return 1;
    }
    if (cur == NULL || cur->get_key() != key) {
        int level = get_random_level();
        // 如果层数大于当前节点的最大层数list_level
        if (level > list_level) {   // 那大于list_level部分的前继就是头节点
            for (int i = list_level + 1; i <= level; i++) {
                update[i] = head;
            }
            list_level = level;
        }
        Node *node = new Node(key, value, level);
        // 新建节点的前继是update[i]，后继是upda[i]->forward[i]
        for (int i = 0; i <= level; i++) {
            node->forward[i] = update[i]->forward[i];   // 初始化后继
            update[i]->forward[i] = node;   // 初始化前继
        }
    }
    cout << "Successfully insert key: " << key << endl;
    elem_count++;
    return 0;
}

void delete_node(int key) {
    Node *cur = head;
    Node *update[MAXLEVEL + 1];
    memset(update, 0, sizeof(Node*) * (MAXLEVEL + 1));
    for (int i = list_level; i >= 0; i--) {
        while (cur->forward[i] && cur->forward[i]->get_key() < key) {
            cur = cur->forward[i];
        }
        update[i] = cur;
    }
    cur = cur->forward[0];
    if (cur != NULL && cur->get_key() == key) {
        for (int i = 0; i <= list_level; i++) {
            if (update[i]->forward[i] != cur) {
                break;  // 没有指向它就不用管了
            }
            update[i]->forward[i] = cur->forward[i];
        }

        // 如果删掉这个节点后这层就没有了，那当前最大层数就要减小
        while (list_level > 0 && head->forward[list_level] == NULL) {
            list_level--;
        }
        cout << "Successfully deleted key: " << key << endl;
        elem_count--;
    }
}

void test1() {
    // 测试当前文件里面的SkipList相关的一些函数
    Node *n1 = new Node(1, 100, 1);
    Node *n2 = new Node(2, 200, 0);
    Node *n3 = new Node(4, 400, 2);
    Node *n4 = new Node(5, 500, 0);
    list_level = 2;
    elem_count = 4;
    head->forward[0] = n1;
    head->forward[1] = n1;
    head->forward[2] = n3;
    n1->forward[0] = n2;
    n1->forward[1] = n3;
    n2->forward[0] = n3;
    n3->forward[0] = n4;

    display();

    search_node(4);
    insert_node(3, 300);
    display();

    delete_node(3);
    delete_node(4);
    display();
}

void test2() {
    // 测试test.h里面的SkipList<int,int>
    SkipList skiplist(3);
    skiplist.insert_element(1, 100);
    skiplist.insert_element(2, 200);
    skiplist.insert_element(3, 300);
    skiplist.insert_element(4, 400);
    skiplist.insert_element(5, 500);
    skiplist.display_list();
    skiplist.search_element(3);
    skiplist.search_element(10);
    skiplist.delete_element(3);
    skiplist.search_element(3);
    skiplist.display_list();
    cout << skiplist.size() << endl;
}

int main(int argc, char const *argv[])
{
    cout << ">>>>>>>>>>>>>>>>>>>>> start <<<<<<<<<<<<<<<<<<< " << endl;
    // test1();
    // test2();
    return 0;
}
