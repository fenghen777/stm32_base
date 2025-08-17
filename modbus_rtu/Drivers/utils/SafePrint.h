#ifndef PRINT_H
#define PRINT_H

#include "FreeRTOS.h"
#include "semphr.h"

// Maximum print buffer size
#define PRINT_BUFFER_SIZE 512

// Maximum number of pre-allocated buffers
#define MAX_PRINT_BUFFERS 15

typedef struct {
    char data[PRINT_BUFFER_SIZE];
    size_t length;
} PrintMessage;

// Initialization function
void print_daemon_init(void);

// Print interface used in normal tasks
int printf_rtos(const char* format, ...);

// Print interface used in interrupt service routines
int printf_rtos_isr(const char* format, ...);

#endif // PRINT_H