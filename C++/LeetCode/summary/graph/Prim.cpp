#include <iostream>
#include <cstring>
using namespace std;

const int N = 1005;
const int inf = 0x3f3f3f3f;

int G[N][N], closest[N], lowcost[N], res[N];
bool used[N];
int n, m;
int u, v, w;
int prim() {
    int res = 0;
    used[1] = true;
    lowcost[1] = 0;
    for (int i = 2; i <= n; i++) {  // 初始化
        lowcost[i] = G[1][i];
        closest[i] = 1;
        used[i] = false;
    }
    for (int i = 1; i < n; i++) {
        int tmp = inf;
        int index = 1;
        for (int j = 1; j <= n; j++) {
            if (!used[j] && lowcost[j] < tmp) {
                index = j;
                tmp = lowcost[j];
            }
        }
        if (index == 1) return -1;
        res += lowcost[index];
        used[index] = true;
        for (int j = 1; j <= n; j++) {
            if (!used[j] && G[index][j] < lowcost[j]) {
                lowcost[j] = G[index][j];
                closest[j] = index;
            }
        }
    }
    return res;
}

int main() {
    memset(G, 0x3f, sizeof(G));
    cin >> n >> m;
    for (int i = 0; i < m; i++) {
        cin >> u >> v >> w;
        G[u][v] = w;
        G[v][u] = w;
    }
    for (int i = 1; i <= n; i++) G[i][i] = 0;
    cout << prim() << endl;
    for (int i = 1; i <= n; i++) cout << closest[i] << ' ';
    cout << endl;
    for (int i = 1; i <= n; i++) cout << lowcost[i] << ' ';
    cout << endl;
    return 0;
}
// 5 6
// 1 2 1
// 1 4 4
// 2 3 2
// 2 5 5
// 3 4 3
// 4 5 6
// 11
// 0 1 2 3 2 
// 0 1 2 3 5 