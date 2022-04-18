#include <iostream>
#include "test_2.h"
using namespace std;

void test3() {
    // 测试test_2.h里面的SkipList<K, V>
    SkipList<int, string> skiplist(5);
    skiplist.insert_element(1, "kavin");
    skiplist.insert_element(2, "jack");
    skiplist.insert_element(3, "lisa");
    skiplist.insert_element(4, "lili");
    skiplist.insert_element(5, "ck");
    skiplist.display_list();

    skiplist.search_element(3);
    skiplist.search_element(10);

    skiplist.delete_element(3);
    skiplist.search_element(3);
    skiplist.display_list();

}

int main(int argc, char const *argv[])
{
    test3();
    return 0;
}
