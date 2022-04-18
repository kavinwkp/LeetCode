#include <iostream>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <mutex>
#include <fstream>

using namespace std;

std::mutex mtx;

template<typename K, typename V>
class Node {
public:
    Node() {}
    Node(K k, V v, int);
    ~Node();
    K get_key() const;
    V get_value() const;
    void set_value(V);
    Node<K, V>** forward;
    int node_level;
private:
    K key;
    V value;
};

template<typename K, typename V>
Node<K, V>::Node(K k, V v, int level) {
    key = k;
    value = v;
    node_level = level;
    forward = new Node<K, V>* [level + 1];
    memset(forward, 0, sizeof(Node<K, V>*) * (level + 1));
}

template<typename K, typename V>
Node<K, V>::~Node() {
    delete[] forward;
}

template<typename K, typename V>
K Node<K, V>::get_key() const {
    return key;
}

template<typename K, typename V>
V Node<K, V>::get_value() const {
    return value;
}

template<typename K, typename V>
void Node<K, V>::set_value(V v) {
    value = v;
}

template<typename K, typename V>
class SkipList {
public:
    SkipList(int);
    ~SkipList();
    int get_random_level();
    Node<K, V>* create_node(K, V, int);
    int insert_element(K, V);
    void display_list();
    bool search_element(K);
    void delete_element(K);
    int size();
private:
    int _max_level;
    int _skip_list_level;
    Node<K, V>* _header;
    int _element_count;
};

// 创建一个节点
template<typename K, typename V>
Node<K, V>* SkipList<K, V>::create_node(K k, V v, int level) {
    Node<K, V> *node = new Node<K, V>(k, v, level);
    return node;
}

// 插入一个节点
template<typename K, typename V>
int SkipList<K, V>::insert_element(K key, V value) {
    Node<K, V> *cur = _header;
    Node<K, V> *update[_max_level + 1];
    memset(update, 0, sizeof(Node<K, V>*) * (_max_level + 1));
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
        Node<K, V> *insert_node = create_node(key, value, random_level);
        for (int i = 0; i <= random_level; i++) {
            insert_node->forward[i] = update[i]->forward[i];
            update[i]->forward[i] = insert_node;
        }
        cout << "Successfully insert key: " << key << endl;
        _element_count++;
    }
    return 0;
}

// 输出跳表
template<typename K, typename V>
void SkipList<K, V>::display_list() {
    std::cout << "**********SkipList**********" << std::endl;
    for (int i = 0; i <= _skip_list_level; i++) {
        Node<K, V> *cur = _header->forward[i];
        std::cout << "Level " << i << ": ";
        while (cur) {
            cout << "[" << cur->get_key() << ":" << cur->get_value() << "] ";
            cur = cur->forward[i];
        }
        std::cout << std::endl;
    }
}


template<typename K, typename V>
SkipList<K, V>::SkipList(int max_level) {
    _max_level = max_level;
    _skip_list_level = 0;
    _element_count = 0;
    K k;
    V v;
    _header = new Node<K, V>(k, v, _max_level);   // 虚拟头结点
}

template<typename K, typename V>
SkipList<K, V>::~SkipList() {
    delete _header; // 只释放了头结点
}


// 跳表大小
template<typename K, typename V>
int SkipList<K, V>::size() {
    return _element_count;
}

// 随机获取一个层
template<typename K, typename V>
int SkipList<K, V>::get_random_level() {
    int k = 1;
    while (rand() % 2) {
        k++;
    }
    k = (k < _max_level) ? k : _max_level;
    return k;
}

// 搜索元素
template<typename K, typename V>
bool SkipList<K, V>::search_element(K key) {
    std::cout << "-----------search_element-----------" << std::endl;
    Node<K, V> *cur = _header;
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

// 删除一个节点
template<typename K, typename V>
void SkipList<K, V>::delete_element(K key) {
    Node<K, V> *cur = _header;
    Node<K, V> *update[_max_level + 1];   // 记录待删除节点不同层的前继节点
    memset(update, 0, sizeof(Node<K, V>*) * (_max_level + 1));
    
    for (int i = _skip_list_level; i >= 0; i--) {
        while (cur->forward[i] && cur->forward[i]->get_key() < key) {
            cur = cur->forward[i];
        }
        update[i] = cur;
    }
    cur = cur->forward[0];
    if (cur != NULL && cur->get_key() == key) { // 找到对应节点，删除之
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