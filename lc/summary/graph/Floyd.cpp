#include <iostream>
#include <cstring>

using namespace std;

const int N = 1005;
const int M = 10005;
const int inf = 0x3f3f3f3f;

int G[N][N], pre[N][N], dist[N][N];

int n, m;
int u, v, w;

void Floyd() {
    for (int i = 1; i <= n; i++) {
        for (int j = 1; j <= n; j++) {
            if (i == j) dist[i][j] = 0;
            else dist[i][j] = G[i][j];
            if (dist[i][j] != inf && i != j) pre[i][j] = i;
            else pre[i][j] = -1;
        }
    }
    // for (int i = 1; i <= n; i++) {
    //     for (int j = 1; j <= n; j++) {
    //         cout << dist[i][j] << ' ';
    //     }
    //     cout << endl;
    // }
    for (int k = 1; k <= n; k++) {  // 把k节点插入到ij之间
        for (int i = 1; i <= n; i++) {
            for (int j = 1; j <= n; j++) {
                if (dist[i][j] > dist[i][k] + dist[k][j]) {
                    dist[i][j] = dist[i][k] + dist[k][j];
                    pre[i][j] = pre[k][j];
                }
            }
        }
    }
}

void findPath(int i, int j) {
    if (pre[i][j] == -1) return;
    findPath(i, pre[i][j]);
    cout << pre[i][j] << "->";
}

int main(int argc, char const *argv[])
{
    memset(G, inf, sizeof(G));
    cin >> n >> m;
    for (int i = 0; i < m; i++) {
        cin >> u >> v >> w;
        G[u][v] = min(G[u][v], w);  // 防止重边覆盖
    }
    Floyd();
    for (int i = 1; i <= n; i++) {
        for (int j = 1; j <= n; j++) {
            cout << dist[i][j] << ' ';
        }
        cout << endl;
    }
    cout << endl;
    for (int i = 1; i <= n; i++) {
        for (int j = 1; j <= n; j++) {
            cout << pre[i][j] << ' ';
        }
        cout << endl;
    }
    cout << endl;
    for (int i = 1; i <= n; i++) {
        for (int j = i + 1; j <= n; j++) {
            findPath(i, j);
            cout << j << endl;
        }
    }
    return 0;
}
// 4 6
// 1 2 1
// 1 4 4
// 2 4 2
// 3 1 3
// 3 2 5
// 4 3 6

// 0 1 9 3 
// 11 0 8 2 
// 3 4 0 6 
// 9 10 6 0 

// -1 1 4 2 
// 3 -1 4 2 
// 3 1 -1 2 
// 3 1 4 -1 

// 1->2
// 1->2->4->3
// 1->2->4
// 2->4->3
// 2->4
// 3->1->2->4
