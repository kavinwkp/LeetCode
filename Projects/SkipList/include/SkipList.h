#include <iostream>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <mutex>
#include <fstream>
#include <vector>

#define STORE_FILE "../store/dumpFile"

std::mutex mtx;
std::string delimiter = ":";

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
    void display_list() const;
    V search_element(K) const;
    void delete_element(K);
    int size() const;
    void dump_file();
    void load_file();
    std::vector<std::pair<K, V>> get_element();
private:
    void get_key_value_from_string(const std::string& str, std::string *key, std::string *value);
    bool is_valid_string(const std::string& str);
private:
    int _max_level;
    int _skip_list_level;
    Node<K, V>* _header;
    int _element_count;
    std::ofstream _file_writer;
    std::ifstream _file_reader;
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
    std::cout << "-----------insert_element-----------" << std::endl;
    mtx.lock();
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
        std::cout << "key: " << key << ", exists" << "\n";
        mtx.unlock();
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
        std::cout << "Successfully insert key: " << key << "\n";
        _element_count++;
    }
    mtx.unlock();
    return 0;
}

// 输出跳表
template<typename K, typename V>
void SkipList<K, V>::display_list() const {
    std::cout << "**********SkipList**********" << "\n";
    for (int i = 0; i <= _skip_list_level; i++) {
        Node<K, V> *cur = _header->forward[i];
        std::cout << "Level " << i << ": ";
        while (cur) {
            std::cout << "[" << cur->get_key() << ":" << cur->get_value() << "] ";
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
    Node<K, V>* cur = _header->forward[0];
    Node<K, V>* tmp = _header->forward[0];
    while (cur) {
        tmp = cur->forward[0];
        std::cout << "delete key:" << cur->get_key() << std::endl;
        delete cur;
        cur = tmp;
    }
    delete _header; // 只释放了头结点
}


// 跳表大小
template<typename K, typename V>
int SkipList<K, V>::size() const {
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
V SkipList<K, V>::search_element(K key) const {
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
        return cur->get_value();
    }
    std::cout << "Not Found Key: " << key << std::endl;
    return "";
}

// 删除一个节点
template<typename K, typename V>
void SkipList<K, V>::delete_element(K key) {
    std::cout << "-----------delete_element-----------" << std::endl;
    mtx.lock();
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
    mtx.unlock();
    return;
}

// 数据落盘
template<typename K, typename V>
void SkipList<K, V>::dump_file() {
    std::cout << "----------dump_file----------" << "\n";
    _file_writer.open(STORE_FILE);
    Node<K, V> *node = _header->forward[0];
    while (node != NULL) {
        _file_writer << node->get_key() << ":" << node->get_value() << std::endl;
        node = node->forward[0];
    }
    _file_writer.flush();
    _file_writer.close();
    return;
}

// 从文件读取键值对
template<typename K, typename V>
void SkipList<K, V>::load_file() {
    std::cout << "----------load_file----------" << "\n";
    _file_reader.open(STORE_FILE);
    std::string line;
    std::string *key = new std::string();
    std::string *value = new std::string();
    while (getline(_file_reader, line)) {
        get_key_value_from_string(line, key, value);
        std::cout << *key << ":" << *value << "\n";
        if (key->empty() || value->empty()) {
            continue;
        }
        insert_element(*key, *value);
    }
    _file_reader.clear();
    delete key;
    delete value;
}

// 从字符串获取键值对
template<typename K, typename V>
void SkipList<K, V>::get_key_value_from_string(const std::string& str, std::string *key, std::string *value) {
    if (!is_valid_string(str)) return;
    *key = str.substr(0, str.find(delimiter));
    *value = str.substr(str.find(delimiter) + 1, str.length());
}

// 判断字符串是否有效
template<typename K, typename V>
bool SkipList<K, V>::is_valid_string(const std::string& str) {
    if (str.empty()) return false;
    if (str.find(delimiter) == std::string::npos) return false;
    return true;
}

template<typename K, typename V>
std::vector<std::pair<K, V>> SkipList<K, V>::get_element() {
    Node<K, V> *cur = _header->forward[0];
    std::vector<std::pair<K, V>> slvec;
    while (cur) {
        slvec.emplace_back(cur->get_key(), cur->get_value());
        cur = cur->forward[0];
    }
    return slvec;
}