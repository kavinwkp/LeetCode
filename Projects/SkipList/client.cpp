#include <stdio.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>

int main() {
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in serveraddr;
    serveraddr.sin_family = AF_INET;
    inet_pton(AF_INET, "127.0.0.1", &serveraddr.sin_addr.s_addr);
    serveraddr.sin_port = htons(9999);
    int ret = connect(fd, (struct sockaddr*)&serveraddr, sizeof(serveraddr));

    char recvBuf[1024] = {0};

    while (1) {
        fgets(recvBuf, 1024, stdin);
        recvBuf[strlen(recvBuf) - 1] = '\0';
        write(fd, recvBuf, strlen(recvBuf) + 1);
        int len = read(fd, recvBuf, sizeof(recvBuf));
        if (len == -1) {
            perror("read");
            exit(-1);
        }
        else if (len > 0) {
            printf("%s\n", recvBuf);
        }
        else if (len == 0) {
            printf("server close\n");
            break;
        }
        sleep(1);
    }
    close(fd);
    return 0;
}