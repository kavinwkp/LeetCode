#include <iostream>

#include "../include/Algorithm.h"
// #include "Profiler\Profiler.h"

#include "../include/AlgorithmTest.h"
// #include "AVLTreeTest.h"
// #include "BitmapTest.h"
// #include "BinarySearchTreeTest.h"
// #include "CircularBufferTest.h"
// #include "COWPtrTest.h"
// #include "DequeTest.h"
// #include "GraphTest.h"
// #include "ListTest.h"
// #include "Test\PairTest.h"
// #include "PriorityQueueTest.h"
// #include "Test\QueueTest.h"
// #include "Test\RefTest.h"
// #include "Test\SharedPtrTest.h"
// #include "Test\StackTest.h"
// #include "Test\StringTest.h"
// #include "Test\SuffixArrayTest.h"
// #include "Test\TrieTreeTest.h"
// #include "Test\TypeTraitsTest.h"
// #include "Test\UFSetTest.h"
// #include "Test\UniquePtrTest.h"
// #include "Test\Unordered_setTest.h"
#include "../include/VectorTest.h"

// using namespace TinySTL::Profiler;
using std::cout;
using std::endl;

struct Stone {
	int m;
	Stone() { m = 0; }
	Stone(int _m): m(_m) {}
};

int main(){
	// TinySTL::AlgorithmTest::testFill();
	TinySTL::VectorTest::testCase1();
	// TinySTL::VectorTest::testCase2();
	// TinySTL::VectorTest::testCase3();
	// TinySTL::VectorTest::testCase4();

	// TinySTL::vector<Stone> nums(10);
	// TinySTL::uninitialized_fill_n(nums.begin(), 5, Stone(5));
	// for (int i = 0; i < nums.size(); i++) {
	// 	cout << nums[i].m << ' ';
	// }
	// cout << endl;


	// TinySTL::AlgorithmTest::testAllCases();
	// TinySTL::AVLTreeTest::testAllCases();
	// TinySTL::BitmapTest::testAllCases();
	// TinySTL::BinarySearchTreeTest::testAllCases();
	// TinySTL::CircularBufferTest::testAllCases();
	// TinySTL::COWPtrTest::testAllCases();
	// TinySTL::DequeTest::testAllCases();
	// TinySTL::ListTest::testAllCases();
	// TinySTL::GraphTest::testAllCases();
	// TinySTL::PairTest::testAllCases();
	// TinySTL::PriorityQueueTest::testAllCases();
	// TinySTL::QueueTest::testAllCases();
	// TinySTL::RefTest::testAllCases();
	// TinySTL::SharedPtrTest::testAllCases();
	// TinySTL::StackTest::testAllCases();
	// TinySTL::StringTest::testAllCases();
	// TinySTL::SuffixArrayTest::testAllCases();
	// TinySTL::TrieTreeTest::testAllCases();
	// TinySTL::TypeTraitsTest::testAllCases();
	// TinySTL::UFSetTest::testAllCases();
	// TinySTL::UniquePtrTest::testAllCases();
	// TinySTL::Unordered_setTest::testAllCases();
	// TinySTL::VectorTest::testAllCases();
	 
	// system("pause");
	return 0;
}