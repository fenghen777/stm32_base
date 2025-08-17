#include "TcpServer.h"
#include "lwip/netif.h"
#include "lwip/init.h"
#include "netif/etharp.h"
#include <stdio.h>
#include <string.h>
#include "lwip/sockets.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

static int set_nonblocking(int fd) {
    int flags = fcntl(fd, F_GETFL, 0);
    return fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

void handle_client_task(void* pvParameters) {
    printf_rtos("[服务端] 开始处理客户端连接\n");
    int connfd = (int) pvParameters;
    char* buff = (char*) pvPortMalloc(512);
    if (buff == NULL) {
        // 内存分配失败处理（优先级：记录 > 降级 > 复位）
        printf_rtos("[ERR] 内存分配失败！剩余堆：%d字节\n", xPortGetFreeHeapSize());
        close(connfd);
        vTaskDelete(NULL); // 关闭当前任务
        return;
    }
    memset(buff, 0, 512);
    int n;
    while (1) {
        n = read(connfd, buff, 512 - 1);
        if (n > 0) {
            buff[n] = '\0';
            printf_rtos("Received: %s\n", buff);
            write(connfd, buff, n); // 回显给客户端
        } else if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // 避免阻塞等 让出CPU执行时间
                osDelay(pdMS_TO_TICKS(100));
            } else {
                // 连接关闭或错误发生
                printf_rtos("[服务端] 连接关闭或错误发生 %d\n", n);
                break;
            }

        }
    }
    vPortFree(buff);
    buff = NULL;
    close(connfd);
    vTaskDelete(NULL); // 关闭当前任务
}

void start_tcp_server_multi_task() {
    printf_rtos("[服务端] 开始监听\n");
    int listenfd = socket(AF_INET, SOCK_STREAM, 0);
    // 设置地址重用
    int opt = 1;
    setsockopt(listenfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in serv_addr;

    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    serv_addr.sin_port = htons(TCP_SERVER_PORT);

    bind(listenfd, (struct sockaddr*) &serv_addr, sizeof(serv_addr));
    listen(listenfd, MAX_TCP_CLIENTS);

    while (1) {
        // 2 3参数获取客户端的地址信息
        int connfd = accept(listenfd, (struct sockaddr*) NULL, NULL);
        if (connfd >= 0) {
            printf_rtos("[服务端] 收到客户端连接\n");
//            struct timeval timeout;
//            timeout.tv_sec = 1;
//            timeout.tv_usec = 0;
//            setsockopt(connfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
            set_nonblocking(connfd);
            // 创建新任务处理客户端连接
            BaseType_t xReturned;
            xReturned = xTaskCreate(handle_client_task, "Handle Client", 512, (void*) connfd, 5, NULL);
            printf_rtos("剩余堆内存：%d字节\n", xPortGetFreeHeapSize());
            if (xReturned != pdPASS) {
                printf_rtos("[服务端] 创建任务失败 %d\n", (int) xReturned);
            }
        } else {
            printf_rtos("[服务端] 监听失败 sleep\n");
            osDelay(pdMS_TO_TICKS(100));
        }
        printf_rtos("[服务端] accept循环\n");
    }
}

static int client_socks[MAX_TCP_CLIENTS] = {0};

void start_tcp_server_select() {
    int server_fd, max_fd, activity;
    struct sockaddr_in server_addr;
    fd_set readfds;

    // 创建TCP socket
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        printf_rtos("Socket creation error\n");
        vTaskDelete(NULL);
    }

    // 设置地址重用
    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // 绑定地址
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(TCP_SERVER_PORT);
    server_addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(server_fd, (struct sockaddr*) &server_addr, sizeof(server_addr)) < 0) {
        printf_rtos("Bind failed\n");
        close(server_fd);
        vTaskDelete(NULL);
    }

    // 设置非阻塞模式
    fcntl(server_fd, F_SETFL, O_NONBLOCK);

    // 开始监听
    if (listen(server_fd, MAX_TCP_CLIENTS) < 0) {
        printf_rtos("Listen failed\n");
        close(server_fd);
        vTaskDelete(NULL);
    }

    printf_rtos("TCP Server started on port %d\n", TCP_SERVER_PORT);

    while (1) {
        FD_ZERO(&readfds);
        FD_SET(server_fd, &readfds);
        max_fd = server_fd;

        // 添加客户端socket到监控集合
        for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
            if (client_socks[i] > 0) {
                FD_SET(client_socks[i], &readfds);
                if (client_socks[i] > max_fd)
                    max_fd = client_socks[i];
            }
        }

        // 设置超时
        struct timeval tv = {
                .tv_sec = SELECT_TIMEOUT_MS / 1000,
                .tv_usec = (SELECT_TIMEOUT_MS % 1000) * 1000
        };

        activity = select(max_fd + 1, &readfds, NULL, NULL, &tv);
        if (activity < 0 && errno != EINTR) {
            printf_rtos("Select error\n");
        }

        // 处理新连接
        if (FD_ISSET(server_fd, &readfds)) {
            struct sockaddr_in cli_addr;
            socklen_t clilen = sizeof(cli_addr);
            int new_sock = accept(server_fd, (struct sockaddr*) &cli_addr, &clilen);

            if (new_sock < 0) {
                printf_rtos("Accept error\n");
                continue;
            }

            // 添加到客户端数组
            for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
                if (client_socks[i] == 0) {
                    client_socks[i] = new_sock;
                    fcntl(new_sock, F_SETFL, O_NONBLOCK);
                    printf_rtos("New client: %d\n", new_sock);
                    break;
                }
            }
        }

        // 处理客户端数据
        for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
            int sockfd = client_socks[i];
            if (sockfd <= 0 || !FD_ISSET(sockfd, &readfds))
                continue;

            char buffer[TCP_BUFFER_SIZE];
            ssize_t valread = read(sockfd, buffer, sizeof(buffer) - 1);

            if (valread > 0) {
                buffer[valread] = '\0';
                printf_rtos("Recv %d bytes: %s\n", valread, buffer);
                write(sockfd, buffer, valread); // 回显数据
            } else if (valread == 0 || (valread < 0 && errno != EAGAIN)) {
                printf_rtos("Client %d disconnected\n", sockfd);
                close(sockfd);
                client_socks[i] = 0;
            }
        }
        taskYIELD(); // 释放CPU
    }
}





























