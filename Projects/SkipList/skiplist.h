/* ************************************************************************
> File Name:     skiplist.h
> Author:        程序员Carl
> 微信公众号:    代码随想录
> Created Time:  Sun Dec  2 19:04:26 2018
> Description:   
 ************************************************************************/

#include <iostream> 
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <mutex>
#include <fstream>

#define STORE_FILE "store/dumpFile"

std::mutex mtx;     // mutex for critical section
std::string delimiter = ":";

//Class template to implement node
template<typename K, typename V> 
class Node {    // 跳表节点类

public:
    
    Node() {} // 默认构造函数

    Node(K k, V v, int); // 自定义构造函数

    ~Node();    // 析构函数

    K get_key() const;  // 获取key值

    V get_value() const;    // 获取value值

    void set_value(V);  // 设置value值
    
    // Linear array to hold pointers to next node of different level
    Node<K, V> **forward;   // 指针数组，每个元素指向下一层的下一个节点

    int node_level;     // 当前层？有点疑问

private:
    K key;      // 私有成员：K类型的key
    V value;    // 私有成员：V类型的value
};

template<typename K, typename V> 
Node<K, V>::Node(const K k, const V v, int level) {
    this->key = k;
    this->value = v;
    this->node_level = level;   // 一开始就指定层？

    // level + 1, because array index is from 0 - level
    this->forward = new Node<K, V>*[level+1];    // level=2的话就分配3个指针，第0个指向后面的元素，第1个是第一层，第2个是第二层
    
	// Fill forward array with 0(NULL) 
    memset(this->forward, 0, sizeof(Node<K, V>*)*(level+1));
};

template<typename K, typename V> 
Node<K, V>::~Node() {
    delete []forward;   // 把指针数组都delete[]
};

template<typename K, typename V> 
K Node<K, V>::get_key() const {
    return key;
};

template<typename K, typename V> 
V Node<K, V>::get_value() const {
    return value;
};
template<typename K, typename V> 
void Node<K, V>::set_value(V value) {
    this->value=value;
};

// Class template for Skip list
template <typename K, typename V> 
class SkipList {    // 跳表类

public: 
    SkipList(int);              // 构造函数，输入一个整数代表最大层数
    ~SkipList();                // 析构函数
    int get_random_level();     // 随机获取层
    Node<K, V>* create_node(K, V, int); // 创建一个节点，返回节点指针
    int insert_element(K, V);   // 插入元素
    void display_list();        // 跳表输出
    bool search_element(K);     // 查找元素
    void delete_element(K);     // 删除元素
    void dump_file();
    void load_file();
    int size();                 // 跳表大小

private:
    void get_key_value_from_string(const std::string& str, std::string* key, std::string* value);
    bool is_valid_string(const std::string& str);

private:    
    // Maximum level of the skip list
    int _max_level;     // 最大层数，一开始就确定了

    // current level of skip list 
    int _skip_list_level;   // 当前层

    // pointer to header node 
    Node<K, V> *_header;    // 指向头结点的指针，层数为最大_max_level

    // file operator
    std::ofstream _file_writer;
    std::ifstream _file_reader;

    // skiplist current element count
    int _element_count;     // 跳表中有几个节点Node
};

// create new node 
template<typename K, typename V>
Node<K, V>* SkipList<K, V>::create_node(const K k, const V v, int level) {
    Node<K, V> *n = new Node<K, V>(k, v, level);
    return n;
}

// Insert given key and value in skip list 
// return 1 means element exists  
// return 0 means insert successfully
/* 
                           +------------+
                           |  insert 50 |
                           +------------+
level 4     +-->1+                                                      100
                 |
                 |                      insert +----+
level 3         1+-------->10+---------------> | 50 |          70       100
                                               |    |
                                               |    |
level 2         1          10         30       | 50 |          70       100
                                               |    |
                                               |    |
level 1         1    4     10         30       | 50 |          70       100
                                               |    |
                                               |    |
level 0         1    4   9 10         30   40  | 50 |  60      70       100
                                               +----+

*/
template<typename K, typename V>
int SkipList<K, V>::insert_element(const K key, const V value) {
    
    mtx.lock();     // 加锁
    Node<K, V> *current = this->_header;

    // create update array and initialize it 
    // update is array which put node that the node->forward[i] should be operated later
    Node<K, V> *update[_max_level+1];
    memset(update, 0, sizeof(Node<K, V>*)*(_max_level+1));  

    // start form highest level of skip list 
    for(int i = _skip_list_level; i >= 0; i--) {    // 从最高层开始，找到小于key的第一个节点
        while(current->forward[i] != NULL && current->forward[i]->get_key() < key) {
            current = current->forward[i]; 
        }
        update[i] = current;    // 把每层的待插入元素的前继节点保存起来，待会要操作他们的指针
    }
    // 到这里cur->40
    // reached level 0 and forward pointer to right node, which is desired to insert key.
    current = current->forward[0];  // 此时cur->60

    // if current node have key equal to searched key, we get it
    if (current != NULL && current->get_key() == key) {
        std::cout << "key: " << key << ", exists" << std::endl;
        mtx.unlock();
        return 1;
    }

    // if current is NULL that means we have reached to end of the level 
    // if current's key is not equal to key that means we have to insert node between update[0] and current node 
    if (current == NULL || current->get_key() != key ) {
        
        // Generate a random level for node
        int random_level = get_random_level();  // 获取一个随机的层数

        // If random level is greater thar skip list's current level, initialize update value with pointer to header
        if (random_level > _skip_list_level) {  // 获取的层数如果大于当前的层数
            for (int i = _skip_list_level+1; i < random_level+1; i++) {
                update[i] = _header;    // 超过当前层数的那些要称为头结点的后继
            }
            _skip_list_level = random_level;    // 更新当前层数
        }

        // create new node with random level generated 
        Node<K, V>* inserted_node = create_node(key, value, random_level);
        
        // insert node 
        for (int i = 0; i <= random_level; i++) {   // 插入节点
            inserted_node->forward[i] = update[i]->forward[i];  // 新节点的前继节点的后继节点变成新节点的后继节点
            update[i]->forward[i] = inserted_node;  // 新节点的前继节点的后继节点变成新节点
        }
        std::cout << "Successfully inserted key:" << key << ", value:" << value << std::endl;
        _element_count ++;  // 节点数增加一个
    }
    mtx.unlock();   // 解锁
    return 0;
}

// Display skip list 
template<typename K, typename V> 
void SkipList<K, V>::display_list() {

    std::cout << "\n*****Skip List*****"<<"\n"; 
    for (int i = 0; i <= _skip_list_level; i++) {
        Node<K, V> *node = this->_header->forward[i]; 
        std::cout << "Level " << i << ": ";
        while (node != NULL) {
            std::cout << node->get_key() << ":" << node->get_value() << ";";
            node = node->forward[i];
        }
        std::cout << std::endl;
    }
}

// Dump data in memory to file 
template<typename K, typename V> 
void SkipList<K, V>::dump_file() {

    std::cout << "dump_file-----------------" << std::endl;
    _file_writer.open(STORE_FILE);
    Node<K, V> *node = this->_header->forward[0]; 

    while (node != NULL) {
        _file_writer << node->get_key() << ":" << node->get_value() << "\n";
        std::cout << node->get_key() << ":" << node->get_value() << ";\n";
        node = node->forward[0];
    }

    _file_writer.flush();
    _file_writer.close();
    return ;
}

// Load data from disk
template<typename K, typename V> 
void SkipList<K, V>::load_file() {

    _file_reader.open(STORE_FILE);
    std::cout << "load_file-----------------" << std::endl;
    std::string line;
    std::string* key = new std::string();
    std::string* value = new std::string();
    while (getline(_file_reader, line)) {
        get_key_value_from_string(line, key, value);
        if (key->empty() || value->empty()) {
            continue;
        }
        insert_element(*key, *value);
        std::cout << "key:" << *key << "value:" << *value << std::endl;
    }
    _file_reader.close();
}

// Get current SkipList size 
template<typename K, typename V> 
int SkipList<K, V>::size() { 
    return _element_count;
}

template<typename K, typename V>
void SkipList<K, V>::get_key_value_from_string(const std::string& str, std::string* key, std::string* value) {

    if(!is_valid_string(str)) {
        return;
    }
    *key = str.substr(0, str.find(delimiter));
    *value = str.substr(str.find(delimiter)+1, str.length());
}

template<typename K, typename V>
bool SkipList<K, V>::is_valid_string(const std::string& str) {

    if (str.empty()) {
        return false;
    }
    if (str.find(delimiter) == std::string::npos) {
        return false;
    }
    return true;
}

// Delete element from skip list 
template<typename K, typename V> 
void SkipList<K, V>::delete_element(K key) {

    mtx.lock();
    Node<K, V> *current = this->_header; 
    Node<K, V> *update[_max_level+1];
    memset(update, 0, sizeof(Node<K, V>*)*(_max_level+1));

    // start from highest level of skip list
    for (int i = _skip_list_level; i >= 0; i--) {
        while (current->forward[i] !=NULL && current->forward[i]->get_key() < key) {
            current = current->forward[i];
        }
        update[i] = current;
    }

    current = current->forward[0];
    if (current != NULL && current->get_key() == key) {
       
        // start for lowest level and delete the current node of each level
        for (int i = 0; i <= _skip_list_level; i++) {

            // if at level i, next node is not target node, break the loop.
            if (update[i]->forward[i] != current) 
                break;

            update[i]->forward[i] = current->forward[i];
        }

        // Remove levels which have no elements
        while (_skip_list_level > 0 && _header->forward[_skip_list_level] == 0) {
            _skip_list_level --;    // 如果当前最高层没有了，就把最高层减一
        }

        std::cout << "Successfully deleted key "<< key << std::endl;
        _element_count --;  // 节点数减少一个
    }
    mtx.unlock();
    return;
}

// Search for element in skip list 
/*
                           +------------+
                           |  select 60 |
                           +------------+
level 4     +-->1+                                                      100
                 |
                 |
level 3         1+-------->10+------------------>50+           70       100
                                                   |
                                                   |
level 2         1          10         30         50|           70       100
                                                   |
                                                   |
level 1         1    4     10         30         50|           70       100
                                                   |
                                                   |
level 0         1    4   9 10         30   40    50+-->60      70       100
*/
template<typename K, typename V> 
bool SkipList<K, V>::search_element(K key) {

    std::cout << "search_element-----------------" << std::endl;
    Node<K, V> *current = _header;

    // start from highest level of skip list
    for (int i = _skip_list_level; i >= 0; i--) {   // 从最高层开始找，直到找到第0层
        while (current->forward[i] && current->forward[i]->get_key() < key) {
            current = current->forward[i];  // 如果cur有后继节点，并且要查找的key大于后继节点值，cur后移
        }
    }
    // 运行到这里的时候cur->50
    //reached level 0 and advance pointer to right node, which we search    
    current = current->forward[0];  // 再往后一个就是cur->60

    // if current node have key equal to searched key, we get it
    if (current and current->get_key() == key) {        // 相等就是找到了
        std::cout << "Found key: " << key << ", value: " << current->get_value() << std::endl;
        return true;
    }

    std::cout << "Not Found Key:" << key << std::endl;  // 否则就是没有
    return false;
}

// construct skip list
template<typename K, typename V> 
SkipList<K, V>::SkipList(int max_level) {

    this->_max_level = max_level;
    this->_skip_list_level = 0;
    this->_element_count = 0;

    // create header node and initialize key and value to null
    K k;
    V v;
    this->_header = new Node<K, V>(k, v, _max_level);   // 初始化一个空的头结点
};

template<typename K, typename V> 
SkipList<K, V>::~SkipList() {

    if (_file_writer.is_open()) {
        _file_writer.close();
    }
    if (_file_reader.is_open()) {
        _file_reader.close();
    }
    delete _header;
}

template<typename K, typename V>
int SkipList<K, V>::get_random_level(){

    int k = 1;
    while (rand() % 2) {    // 奇数就k++
        k++;
    }
    k = (k < _max_level) ? k : _max_level;  // 如果>=最高层就赋值为最高层，否则赋值为k
    return k;
};
// vim: et tw=100 ts=4 sw=4 cc=120
