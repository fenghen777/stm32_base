#include <stdarg.h>
#include "SafePrint.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// Print queue
QueueHandle_t print_queue = NULL;

// DMA transmission semaphore
SemaphoreHandle_t dma_sem = NULL;

// Buffer allocation/release semaphore
SemaphoreHandle_t print_buffer_sem = NULL;

// Buffer pool
static PrintMessage print_buffers[MAX_PRINT_BUFFERS];

static volatile uint8_t buffer_in_use[MAX_PRINT_BUFFERS] = {0};

// Index of buffer currently being sent
static int current_print_buffer_index = -1;

// Get free buffer and fill data
PrintMessage* get_free_print_buffer_and_fill_data(const char* data, size_t length, int* index) {
    xSemaphoreTake(print_buffer_sem, portMAX_DELAY);
    for (int i = 0; i < MAX_PRINT_BUFFERS; ++i) {
        if (!buffer_in_use[i]) {
            buffer_in_use[i] = 1;
            *index = i;
            xSemaphoreGive(print_buffer_sem);

            // Fill data into buffer
            strncpy(print_buffers[i].data, data, length);
            print_buffers[i].length = length;
            print_buffers[i].data[length] = '\0';

            return &print_buffers[i];
        }
    }
    xSemaphoreGive(print_buffer_sem);
    return NULL;
}

PrintMessage* get_free_print_buffer_and_fill_data_isr(const char* data, size_t length, int* index) {
    xSemaphoreTakeFromISR(print_buffer_sem, NULL);
    for (int i = 0; i < MAX_PRINT_BUFFERS; ++i) {
        if (!buffer_in_use[i]) {
            buffer_in_use[i] = 1;
            *index = i;
            xSemaphoreGiveFromISR(print_buffer_sem, NULL);

            // Fill data into buffer
            strncpy(print_buffers[i].data, data, length);
            print_buffers[i].length = length;
            print_buffers[i].data[length] = '\0';

            return &print_buffers[i];
        }
    }
    xSemaphoreGiveFromISR(print_buffer_sem, NULL);
    return NULL;
}

// Release buffer
void release_print_buffer(int index) {
    xSemaphoreTake(print_buffer_sem, portMAX_DELAY);
    if (index >= 0 && index < MAX_PRINT_BUFFERS) {
        buffer_in_use[index] = 0;
    }
    xSemaphoreGive(print_buffer_sem);
}

void release_print_buffer_isr(int index) {
    xSemaphoreTakeFromISR(print_buffer_sem, NULL);
    if (index >= 0 && index < MAX_PRINT_BUFFERS) {
        buffer_in_use[index] = 0;
    }
    xSemaphoreGiveFromISR(print_buffer_sem, NULL);
}

// Daemon task: responsible for putting messages into DMA transmission
static void print_daemon_task(void* arg) {
    (void) arg;
    for (;;) {
        PrintMessage* pmsg = NULL;
        if (xQueueReceive(print_queue, &pmsg, portMAX_DELAY) == pdPASS) {
            int buffer_index = -1;
            for (int i = 0; i < MAX_PRINT_BUFFERS; ++i) {
                if (&print_buffers[i] == pmsg) {
                    buffer_index = i;
                    break;
                }
            }

            // Invalid data, skip
            if (buffer_index == -1) continue;
            if (xSemaphoreTake(dma_sem, portMAX_DELAY) == pdTRUE) {
                // Directly use the incoming message data (already filled in get_free_print_buffer_and_fill_data)
                current_print_buffer_index = buffer_index;

                // Start DMA transmission
                HAL_UART_Transmit_DMA(&huart1, (uint8_t*) pmsg->data, pmsg->length);
            } else {
                // If semaphore cannot be acquired, release the message
                release_print_buffer(buffer_index);
            }
        }
    }
}

// Initialization function
void print_daemon_init(void) {
    print_queue = xQueueCreate(MAX_PRINT_BUFFERS, sizeof(PrintMessage*));
    configASSERT(print_queue);

    dma_sem = xSemaphoreCreateBinary();
    configASSERT(dma_sem);
    xSemaphoreGive(dma_sem); // Initially available

    print_buffer_sem = xSemaphoreCreateBinary();
    configASSERT(print_buffer_sem);
    xSemaphoreGive(print_buffer_sem);

    xTaskCreate(
            print_daemon_task,
            "PrintDaemon",
            configMINIMAL_STACK_SIZE + 512,
            NULL,
            tskIDLE_PRIORITY + 2,
            NULL);
}

// DMA transmission complete callback: only mark current buffer as unused
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (huart == &huart1 && current_print_buffer_index != -1) {
        // Mark current buffer as unused
        release_print_buffer_isr(current_print_buffer_index);
        current_print_buffer_index = -1;

        // Release dma_sem semaphore
        xSemaphoreGiveFromISR(dma_sem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    // todo Handle callbacks for other UARTs
}

// Generic print function (non-ISR)
int printf_rtos(const char* format, ...) {
    va_list args;
    va_start(args, format);

    int buffer_index = -1;
    PrintMessage* pmsg = get_free_print_buffer_and_fill_data(NULL, 0, &buffer_index);
    if (!pmsg) {
        va_end(args);
        return -1;
    }

    int len = vsnprintf(pmsg->data, PRINT_BUFFER_SIZE, format, args);
    va_end(args);

    if (len < 0 || len >= PRINT_BUFFER_SIZE) {
        release_print_buffer(buffer_index);
        return -1;
    }

    pmsg->length = (size_t) len;
    if (xQueueSend(print_queue, &pmsg, portMAX_DELAY) != pdTRUE) {
        release_print_buffer(buffer_index);
        return -2;
    }

    return len;
}

// ISR-safe print function
int printf_rtos_isr(const char* format, ...) {
    va_list args;
    va_start(args, format);

    int buffer_index = -1;
    PrintMessage* pmsg = get_free_print_buffer_and_fill_data_isr(NULL, 0, &buffer_index);
    if (!pmsg) {
        va_end(args);
        return -1;
    }

    int len = vsnprintf(pmsg->data, PRINT_BUFFER_SIZE, format, args);
    va_end(args);

    if (len < 0 || len >= PRINT_BUFFER_SIZE) {
        release_print_buffer_isr(buffer_index);
        return -1;
    }

    pmsg->length = (size_t) len;

    if (xQueueSendFromISR(print_queue, &pmsg, NULL) != pdTRUE) {
        release_print_buffer_isr(buffer_index);
        return -2;
    }
    return len;
}