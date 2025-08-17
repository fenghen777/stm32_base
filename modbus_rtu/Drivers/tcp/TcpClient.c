#include "TcpClient.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include <unistd.h>

int g_client_socket = -1;

// 定义一个标志变量来控制重连次数
int g_reconnect_attempts = 0;

bool g_is_connected = false;

bool is_connected() {
    return g_is_connected;
}

void tcp_client_init(void) {
    struct sockaddr_in serverAddr;
    int enable = 1;
    g_client_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (g_client_socket < 0) {
        printf_rtos("Failed to create socket\n");
        return;
    }
    // 设置 socket 选项, SO_REUSEADDR 允许在服务器关闭后立即重新绑定到同一个端口,无需等待自动释放
    setsockopt(g_client_socket, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));

    // 设置套接字为非阻塞模式
    int flags = fcntl(g_client_socket, F_GETFL, 0);
    if (flags == -1) {
        printf_rtos("Failed to get socket flags\n");
        return;
    }
    flags |= O_NONBLOCK;
    if (fcntl(g_client_socket, F_SETFL, flags) == -1) {
        printf_rtos("Failed to set non-blocking before connect\n");
        closesocket(g_client_socket);
        return;
    }

    serverAddr.sin_family = AF_INET; // IPv4
    serverAddr.sin_port = htons(TCP_CLIENT_PORT); // 端口号，转为网络字节序
    serverAddr.sin_addr.s_addr = inet_addr(SERVER_IP); // IP地址转换
    // 连接到服务器
    int ret = connect(g_client_socket, (struct sockaddr*) &serverAddr, sizeof(serverAddr));
    if (ret < 0 && errno != EINPROGRESS) {
        printf_rtos("Failed to connect to server\n");
        closesocket(g_client_socket);
        g_client_socket = -1;
        g_is_connected = false;
    } else {
        printf_rtos("TCP connection established\n");
        g_is_connected = true;
        g_reconnect_attempts = 0; // 重置重连次数
    }
}

// 读取数据
void tcp_read_data(void) {
    char buffer[256];
    ssize_t bytes_read;
    // 接收
    bytes_read = recv(g_client_socket, buffer, sizeof(buffer) - 1, 0);
    if (bytes_read > 0) {
        buffer[bytes_read] = '\0';
        printf_rtos("Received data: %s\n", buffer);
    } else if (bytes_read == 0) {
        printf_rtos("Connection closed by server\n");
        closesocket(g_client_socket);
        g_client_socket = -1;
        g_is_connected = false;
    } else {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            printf_rtos("No data available (non-blocking mode)\n");
        } else {
            printf_rtos("Error reading data: %d\n", errno);
            closesocket(g_client_socket);
            g_client_socket = -1;
            g_is_connected = false;
        }
    }
}

// 发送数据
void tcp_send_data(char* data) {
    if (g_is_connected) {
        static int count = 0;
        ssize_t bytes_sent = send(g_client_socket, data, strlen(data), 0);
        if (bytes_sent < 0) {
            printf_rtos("Error sending data: %d\n", errno);
            closesocket(g_client_socket);
            g_client_socket = -1;
            g_is_connected = false;
        } else {
            printf_rtos("Sent data: %s\n", data);
        }
    } else {
        printf_rtos("Not connected, cannot send data\n");
    }
}