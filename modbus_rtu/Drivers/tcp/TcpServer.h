#ifndef _TCPECHO_H_
#define _TCPECHO_H_

#define TCP_SERVER_PORT        1030

// 最大连接数
#define MAX_TCP_CLIENTS        10

#define TCP_BUFFER_SIZE        512

#define SELECT_TIMEOUT_MS  100

// 多任务模型
void start_tcp_server_multi_task();

// Select模型
void start_tcp_server_select();

// todo Poll模型

#endif
