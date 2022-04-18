#include <iostream>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <mutex>
#include <fstream>

using namespace std;

std::mutex mtx;

class Node {
public:
    Node() {}
    Node(int k, int v, int);
    ~Node();
    int get_key() const;
    int get_value() const;
    void set_value(int);
    Node** forward;
    int node_level;
private:
    int key;
    int value;
};

inline
Node::Node(int k, int v, int level) {
    key = k;
    value = v;
    node_level = level;
    forward = new Node* [level + 1];
    memset(forward, 0, sizeof(Node*) * (level + 1));
}

inline
Node::~Node() {
    delete[] forward;
}

inline
int Node::get_key() const {
    return key;
}

inline
int Node::get_value() const {
    return value;
}

inline
void Node::set_value(int v) {
    value = v;
}

class SkipList {
public:
    SkipList(int);
    ~SkipList();
    int get_random_level();
    Node* create_node(int, int, int);
    int insert_element(int, int);
    void display_list();
    bool search_element(int);
    void delete_element(int);
    int size();
private:
    int _max_level;
    int _skip_list_level;
    Node* _header;
    int _element_count;
};

inline
SkipList::SkipList(int max_level) {
    _max_level = max_level;
    _skip_list_level = 0;
    _element_count = 0;
    int k = 0;
    int v = 0;
    _header = new Node(k, v, _max_level);   // 虚拟头结点
}

inline
SkipList::~SkipList() {
    delete _header; // 只释放了头结点
}

// 创建一个节点
inline Node*
SkipList::create_node(int k, int v, int level) {
    Node *node = new Node(k, v, level);
    return node;
}

// 跳表大小
inline int
SkipList::size() {
    return _element_count;
}

// 随机获取一个层
inline int
SkipList::get_random_level() {
    int k = 1;
    while (rand() % 2) {
        k++;
    }
    k = (k < _max_level) ? k : _max_level;
    return k;
}

// 搜索元素
inline 
bool SkipList::search_element(int key) {
    std::cout << "-----------search_element-----------" << std::endl;
    Node *cur = _header;
    for (int i = _skip_list_level; i >= 0; i--) {
        while (cur->forward[i] && cur->forward[i]->get_key() < key) {
            cur = cur->forward[i];
        }
    }
    cur = cur->forward[0];
    if (cur && cur->get_key() == key) {
        std::cout << "Found key: " << key << ", value: " << cur->get_value() << std::endl;
        return true;
    }
    std::cout << "Not Found Key: " << key << std::endl;
    return false;
}

// 输出跳表
inline 
void SkipList::display_list() {
    std::cout << "**********SkipList**********" << std::endl;
    for (int i = 0; i <= _skip_list_level; i++) {
        Node *cur = _header->forward[i];
        std::cout << "Level " << i << ": ";
        while (cur) {
            printf("[%d:%d] ", cur->get_key(), cur->get_value());
            cur = cur->forward[i];
        }
        std::cout << std::endl;
    }
}

// 删除一个节点
inline
void SkipList::delete_element(int key) {
    Node *cur = _header;
    Node *update[_max_level + 1];   // 记录待删除节点不同层的前继节点
    memset(update, 0, sizeof(Node*) * (_max_level + 1));
    
    for (int i = _skip_list_level; i >= 0; i--) {
        while (cur->forward[i] && cur->forward[i]->get_key() < key) {
            cur = cur->forward[i];
        }
        update[i] = cur;
    }
    cur = cur->forward[0];
    if (cur && cur->get_key() == key) { // 找到对应节点，删除之
        for (int i = 0; i <= _skip_list_level; i++) {
            if (update[i]->forward[i] != cur) break;
            update[i]->forward[i] = cur->forward[i];   // 处理前继节点的后继
        }
        while (_skip_list_level > 0 && _header->forward[_skip_list_level] == 0) {
            _skip_list_level--;
        }
        std::cout << "Successfully deleted key " << key << std::endl;
        _element_count--;
    }
    return;
}

// 插入一个节点
inline 
int SkipList::insert_element(int key, int value) {
    Node *cur = _header;
    Node *update[_max_level + 1];
    memset(update, 0, sizeof(Node*) * (_max_level + 1));
    for (int i = _skip_list_level; i >= 0; i--) {
        while (cur->forward[i] && cur->forward[i]->get_key() < key) {
            cur = cur->forward[i];
        }
        update[i] = cur;
    }
    cur = cur->forward[0];
    if (cur != NULL && cur->get_key() == key) {
        cout << "key: " << key << ", exists" << endl;
        return 1;
    }
    if (cur == NULL || cur->get_key() != key) {
        int random_level = get_random_level();
        if (random_level > _skip_list_level) {
            for (int i = _skip_list_level + 1; i <= random_level; i++) {
                update[i] = _header;
            }
            _skip_list_level = random_level;
        }
        Node *insert_node = create_node(key, value, random_level);
        for (int i = 0; i <= random_level; i++) {
            insert_node->forward[i] = update[i]->forward[i];
            update[i]->forward[i] = insert_node;
        }
        cout << "Successfully insert key: " << key << endl;
        _element_count++;
    }
    return 0;
}