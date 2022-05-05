#include <iostream>
#include <cstring>
#include <queue>
using namespace std;

const int N = 10005;
const int M = 5e5+5;

struct edge {
    int u, v, w, next;
}e[M];
int cnt;
int dist[N], head[N];
bool inq[N];
void add(int u, int v, int w) {
    e[++cnt].u = u;
    e[cnt].v = v;
    e[cnt].w = w;
    e[cnt].next = head[u];
    head[u] = cnt;
}

int n, m, s;
int u, v, w;

void SPFA(int s) {
    memset(dist, 0x3f, sizeof(dist));
    memset(inq, 0, sizeof(inq));
    queue<int> q;
    q.push(s);
    dist[s] = 0;
    while (!q.empty()) {
        int u = q.front();
        q.pop();
        inq[u] = false;
        for (int i = head[u]; i; i = e[i].next) {
            int v = e[i].v;
            int w = dist[u] + e[i].w;
            if (dist[v] > w) {
                dist[v] = w;
                if (!inq[v]) {
                    inq[v] = true;
                    q.push(v);
                }
            }
        }
    }
}

int main(int argc, char const *argv[])
{
    cin >> n >> m >> s;
    for (int i = 0; i < m; i++) {
        cin >> u >> v >> w;
        add(u, v, w);
    }
    SPFA(s);
    for (int i = 1; i <= n; i++) {
        if (dist[i] != 0x3f3f3f3f) cout << dist[i] << ' ';
        else cout << 2147483647 << ' ';
    }
    cout << endl;
    return 0;
}