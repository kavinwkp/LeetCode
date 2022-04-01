#include <iostream>
#include <cstring>
using namespace std;

const int N = 10005;
const int M = 5e5+5;

struct edge {
    int u, v, w;
}e[M];
int head[N], dist[N];
int cnt;
void add(int u, int v, int w) {
    e[++cnt].u = u;
    e[cnt].v = v;
    e[cnt].w = w;
}
int n, m, s;
int u, v, w;

void Bellman_Ford() {
    memset(dist, 0x3f, sizeof(dist));
    dist[s] = 0;
    for (int i = 1; i < n; i++) {
        bool flag = true;
        for (int j = 1; j <= cnt; j++) {
            int u = e[j].u, v = e[j].v, w = e[j].w;
            if (dist[v] > dist[u] + w) {
                dist[v] = dist[u] + w;
                flag = true;
            }
        }
        if (!flag) break;
    }
}
int main(int argc, char const *argv[])
{
    cin >> n >> m >> s;
    for (int i = 0; i < m; i++) {
        cin >> u >> v >> w;
        add(u, v, w);
    }
    Bellman_Ford();
    for (int i = 1; i <= n; i++) {
        cout << dist[i] << ' ';
    }
    cout << endl;
    return 0;
}