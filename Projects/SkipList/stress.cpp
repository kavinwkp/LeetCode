#include <iostream>
#include <chrono>
#include <cstdlib>
#include <pthread.h>
#include <time.h>
#include "SkipList.h"

using namespace std;

#define NUM_THREADS 1
#define TEST_COUNT 1000000

SkipList<string, string> skiplist(18);

// void* insertElement(void *threadid) {
//     long tid;
//     tid = (long)threadid;
//     cout << tid << endl;
//     int tmp = TEST_COUNT / NUM_THREADS;
//     for (int i = 0; i < tmp; i++) {
//         skiplist.insert_element(rand() % TEST_COUNT, "a");
//     }
//     skiplist.dump_file();
//     pthread_exit(NULL);
// }

void* getElement(void *threadid) {
    long tid;
    tid = (long)threadid;
    cout << tid << endl;
    int tmp = TEST_COUNT / NUM_THREADS;
    for (int i = 0; i < tmp; i++) {
        skiplist.search_element(to_string(rand() % TEST_COUNT));
    }
    pthread_exit(NULL);
}

int main() {
    srand(time(NULL));
    // {
    //     pthread_t threads[NUM_THREADS];
    //     int rc;
    //     auto start = chrono::high_resolution_clock::now();

    //     for (int i = 0; i < NUM_THREADS; i++) {
    //         cout << "main(): creating thread, " << i << endl;
    //         rc = pthread_create(&threads[i], NULL, insertElement, (void*)&i);
    //         if (rc) {
    //             cout << "Err, unable to create thread," << rc << endl;
    //             exit(-1);
    //         }
    //     }

    //     void *ret;
    //     for (int i = 0; i < NUM_THREADS; i++) {
    //         if (pthread_join(threads[i], &ret) != 0) {
    //             perror("pthread_create() error");
    //             exit(3);
    //         }
    //     }
    //     auto finish = std::chrono::high_resolution_clock::now();
    //     std::chrono::duration<double> elapsed = finish - start;
    //     cout << "insert elapsed:" << elapsed.count() << endl;
    // }

    {
        skiplist.load_file();
        pthread_t threads[NUM_THREADS];
        int rc;
        auto start = chrono::high_resolution_clock::now();
        for (int i = 0; i < NUM_THREADS; i++) {
            cout << "main(): creating thread, " << i << endl;
            rc = pthread_create(&threads[i], NULL, getElement, (void*)&i);
            if (rc) {
                cout << "Err, unable to create thread," << rc << endl;
                exit(-1);
            }
        }

        void *ret;
        for (int i = 0; i < NUM_THREADS; i++) {
            if (pthread_join(threads[i], &ret) != 0) {
                perror("pthread_create() error");
                exit(3);
            }
        }
        auto finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = finish - start;
        cout << "insert elapsed:" << elapsed.count() << endl;
    }

    pthread_exit(NULL);
    return 0;
}

// 100000: 0.536052
// 500000: 3.01541
// 1000000: 6.35521
// 每秒可处理写请求数（QPS）: 15.73w

// 10w: 1.17803
// 50w: 6.18252
// 100w: 12.5648
// 每秒可处理写请求数（QPS）: 7.96w