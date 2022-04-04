#include <iostream>
#include <algorithm>
#include <cstring>
using namespace std;

const int N = 1005;
struct edge {
    int u, v, w;
    bool operator<(const edge& rhs) const {
        return w < rhs.w;
    }
}e[N*N];    // 边集数组，方便排序
int cnt;
int fa[N];
int n, m;
int u, v, w;
void add(int u, int v, int w) {
    e[cnt].u = u;
    e[cnt].v = v;
    e[cnt].w = w;
    cnt++;
}
int find(int x) {
    if (x != fa[x]) {
        fa[x] = find(fa[x]);
    }
    return fa[x];
}
bool merge(int a, int b) {
    int p = find(a);
    int q = find(b);
    if (p == q) return false;
    fa[q] = p;
    return true;
}
int kruskal(int n) {
    int res = 0, cnt = 0;
    for (int i = 0; i < m; i++) {
        if (merge(e[i].u, e[i].v)) {
            res += e[i].w;
            cnt++;
            if (cnt == n - 1) return res;
        }
    }
    return -1;
}

int main(int argc, char const *argv[])
{
    cin >> n >> m;
    for (int i = 0; i < m; i++) {
        cin >> u >> v >> w;
        add(u, v, w);
    }
    sort(e, e + cnt);
    // for (int i = 0; i < cnt; i++) {
    //     cout << e[i].u << "->" << e[i].v << ": " << e[i].w << endl;
    // }
    for (int i = 1; i <= n; i++) fa[i] = i;
    cout << kruskal(n) << endl;
    return 0;
}
// 5 6
// 1 2 1
// 1 4 4
// 2 3 2
// 2 5 5
// 3 4 3
// 4 5 6