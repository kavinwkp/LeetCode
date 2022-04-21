#include <stdio.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <pthread.h>
#include <errno.h>
#include <vector>
#include "SkipList.h"

const int BUFSIZE = 1024;

typedef struct sockInfo {
    int fd;
    struct sockaddr_in addr;
    pthread_t tid;
}sockInfo;

sockInfo sockinfos[128];

SkipList<std::string, std::string> skiplist(18);

void split(std::vector<std::string>& strs, std::string& input) {
    std::string path;
    for (auto& c : input) {
        if (c == ' ' && !path.empty()) {
            strs.push_back(path);
            path.clear();
        }
        else if (c != ' ') path.push_back(c);
    }
    strs.push_back(path);
}

void response_display(char *buf) {
    std::vector<std::pair<std::string, std::string>> vec = skiplist.get_element();
    std::string bufStr;
    bufStr.push_back('\r');
    bufStr.push_back('\n');
    for (auto& v : vec) {
        for (auto& c : v.first) {
            bufStr.push_back(c);
        }
        bufStr.push_back('\r');
        bufStr.push_back('\n');

        for (auto& c : v.second) {
            bufStr.push_back(c);
        }
        bufStr.push_back('\r');
        bufStr.push_back('\n');
    }
    strcpy(buf, bufStr.c_str());
}


void response(char *buf) {
    std::string input(buf);
    std::vector<std::string> req;
    split(req, input);
    if (req[0] == "search" && req.size() == 2) {
        std::string ret = skiplist.search_element(req[1]);
        if (ret != "") sprintf(buf, "Found Key: %s, Value: %s", req[1].c_str(), ret.c_str());
        else sprintf(buf, "Not Found Key: %s", req[1].c_str());
    }
    else if (req[0] == "insert" && req.size() == 3) {
        skiplist.insert_element(req[1], req[2]);
        sprintf(buf, "Insert Key: %s, Value: %s", req[1].c_str(), req[2].c_str());
    }
    else if (req[0] == "delete" && req.size() == 2) {
        skiplist.delete_element(req[1]);
        sprintf(buf, "Delete Key: %s", req[1].c_str());
    }
    else if (req[0] == "display" && req.size() == 1) {
        response_display(buf);
    }
    else {
        sprintf(buf, "Invalid Request");
    }
}

void* working(void *arg) {
    sockInfo *pinfo = (sockInfo*)arg;
    char cliIP[16];
    inet_ntop(AF_INET, &pinfo->addr.sin_addr.s_addr, cliIP, sizeof(cliIP));
    unsigned short cliPort = ntohs(pinfo->addr.sin_port);
    printf("client IP: %s, Port: %d\n", cliIP, cliPort);
    char recvBuf[BUFSIZE]; // 接收客户端数据
    while (1) {
        int len = read(pinfo->fd, recvBuf, sizeof(recvBuf));
        if (len == -1) {
            perror("read");
            exit(-1);
        }
        else if (len > 0) {
            printf("Request: %s\n", recvBuf);
            response(recvBuf);
        }
        else if (len == 0) {
            printf("Client Close\n");
            break;
        }
        write(pinfo->fd, recvBuf, strlen(recvBuf) + 1);   // 带上字符串结束符
    }
    close(pinfo->fd);
    return NULL;
}

int main() {
    int lfd = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in saddr;
    saddr.sin_family = AF_INET;
    saddr.sin_port = htons(9999);
    saddr.sin_addr.s_addr = INADDR_ANY;
    int ret = bind(lfd, (struct sockaddr*)&saddr, sizeof(saddr));

    ret = listen(lfd, 128);

    int max = sizeof(sockinfos) / sizeof(sockinfos[0]);
    for (int i = 0; i < max; i++) {
        bzero(&sockinfos[i], sizeof(sockinfos[i]));
        sockinfos[i].fd = -1;
        sockinfos[i].tid = -1;
    }

    while (1) {
        struct sockaddr_in cliaddr;
        socklen_t len = sizeof(cliaddr);
        int cfd = accept(lfd, (struct sockaddr*)&cliaddr, &len);

        sockInfo *pInfo;
        for (int i = 0; i < max; i++) {
            if (sockinfos[i].fd == -1) {
                pInfo = &sockinfos[i];
                break;
            }
        }
        pInfo->fd = cfd;
        memcpy(&pInfo->addr, &cliaddr, len);

        pthread_create(&pInfo->tid, NULL, working, pInfo);
        pthread_detach(pInfo->tid);
    }
    close(lfd);
    return 0;
}

