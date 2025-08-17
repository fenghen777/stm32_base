#ifndef _TCPCLIENT_H_
#define _TCPCLIENT_H_

#include <stdbool.h>

#define SERVER_IP "192.168.10.10"

#define TCP_CLIENT_PORT 1030

// 最大尝试次数
#define MAX_RECONNECT_ATTEMPTS 5

extern int g_reconnect_attempts;

// 初始化
void tcp_client_init(void);

// 判断是否已经连接
bool is_connected();

// 接收数据
void tcp_read_data(void);

// 发送数据
void tcp_send_data(char* data);

#endif
