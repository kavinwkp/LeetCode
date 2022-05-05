#include<cstdio>
#include<iostream>
#include<cstring>
#include<algorithm>
#include<queue>
using namespace std;
const int N=5005;
const int M=200005;

struct edge {
    int v, w, next;
}e[M<<1];

int head[N], dist[N];
bool inq[N];
int cnt;
void add(int u, int v, int w) {
    e[++cnt].v = v;
    e[cnt].w = w;
    e[cnt].next = head[u];
    head[u] = cnt;
}

int n, m;
int u, v, w;
int Prim() {
    memset(dist, 0x3f, sizeof(dist));
    int cnt = 0, res = 0;
    priority_queue<pair<int,int>, vector<pair<int,int>>, greater<pair<int,int>>> q;
    q.push({0, 1});
    dist[1] = 0;
    while (!q.empty()) {
        auto x = q.top();
        q.pop();
        int u = x.second;
        if (inq[u]) continue;
        inq[u] = true;
        cnt++;
        res += dist[u];
        for (int i = head[u]; i; i = e[i].next) {
            int v = e[i].v;
            int w = dist[u] + e[i].w;
            if (dist[v] > w) {
                dist[v] = w;
                q.push({w, v});
            }
        }
    }
    if (cnt < n) return -1;
    return res;
}
int main() {
    cin >> n >> m;
    for (int i = 0; i < m; i++) {
        cin >> u >> v >> w;
        add(u, v, w);
        add(v, u, w);
    }
    cout << Prim();
    return 0;
}