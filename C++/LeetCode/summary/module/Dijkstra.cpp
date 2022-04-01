#include <iostream>
#include <cstring>
#include <algorithm>
#include <queue>

using namespace std;

const int N = 10005;
const int M = 5e5+5;

struct edge {
    int v, w, next;
}e[M];
int cnt;
int head[N], dist[N];
bool inq[N];
void add(int u, int v, int w) {
    e[++cnt].v = v;
    e[cnt].w = w;
    e[cnt].next = head[u];
    head[u] = cnt;
}

void Dijkstra(int s) {
    memset(dist, 0x3f, sizeof(dist));
    priority_queue<pair<int,int>> q;
    q.push({0, s});
    dist[s] = 0;
    while (!q.empty()) {
        auto x = q.top();
        q.pop();
        int u = x.second;
        if (inq[u]) continue;
        inq[u] = true;
        for (int i = head[u]; i; i = e[i].next) {
            int v = e[i].v;
            int w = dist[u] + e[i].w;
            if (dist[v] > w) {
                dist[v] = w;
                q.push({w, v});
            }
        }
    }
}

int main(int argc, char const *argv[])
{
    int n, m, s;
    int u, v, w;
    cin >> n >> m >> s;
    for (int i = 0; i < m; i++) {
        cin >> u >> v >> w;
        add(u, v, w);
    }
    Dijkstra(s);
    for (int i = 1; i <= n; i++) {
        cout << dist[i] << ' ';
    }
    cout << endl;
    return 0;
}