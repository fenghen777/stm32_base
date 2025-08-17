/*
 * mbport_serial.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-01-25     nx           the first version
 *
 */

#include "mbport_serial.h"

#if (MODBUS_SER_ENABLED > 0)

#if (MODBUS_SER_PORT1_ENABLE > 0)
mbport_serial_t mb_serial_1;
#endif
#if (MODBUS_SER_PORT2_ENABLE > 0)
mbport_serial_t mb_serial_2;
#endif
#if (MODBUS_SER_PORT3_ENABLE > 0)
mbport_serial_t mb_serial_3;
#endif
#if (MODBUS_SER_PORT4_ENABLE > 0)
mbport_serial_t mb_serial_4;
#endif

#define MODBUS_USED_RS485 1

#if (MODBUS_USED_RS485 > 0)

void mbport_serif_rs485_init(mbport_serial_t* ser) {
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    /* Initialize the RE pin */
    GPIO_InitStruct.Pin = ser->rs485_pin.pin_re;
    HAL_GPIO_Init(ser->rs485_pin.gpio_re, &GPIO_InitStruct);
    HAL_GPIO_WritePin(ser->rs485_pin.gpio_re, ser->rs485_pin.pin_re,
                      GPIO_PIN_RESET);
    /* Initialize the DE pin */
    GPIO_InitStruct.Pin = ser->rs485_pin.pin_de;
    HAL_GPIO_Init(ser->rs485_pin.gpio_de, &GPIO_InitStruct);
    HAL_GPIO_WritePin(ser->rs485_pin.gpio_de, ser->rs485_pin.pin_de,
                      GPIO_PIN_RESET);
}

#define MB_SER_CFG_RS485_PIN_RE(ser, gpio, pin)     do{ ser->rs485_pin.gpio_re = gpio; ser->rs485_pin.pin_re = pin; } while(0);
#define MB_SER_CFG_RS485_PIN_DE(ser, gpio, pin)     do{ ser->rs485_pin.gpio_de = gpio; ser->rs485_pin.pin_de = pin; } while(0);
#define MB_SER_RS485_PIN_RE(ser, status)        do{ HAL_GPIO_WritePin(ser->rs485_pin.gpio_re, ser->rs485_pin.pin_re, status); } while(0);
#define MB_SER_RS485_PIN_DE(ser, status)        do{ HAL_GPIO_WritePin(ser->rs485_pin.gpio_de, ser->rs485_pin.pin_de, status); } while(0);

#else
#define MB_SER_CFG_RS485_PIN_RE(ser, gpio, pin)
#define MB_SER_CFG_RS485_PIN_DE(ser, gpio, pin)
#define MB_SER_RS485_PIN_RE(ser, status)
#define MB_SER_RS485_PIN_DE(ser, status)

#define mbport_serif_rs485_init(ser)

#endif

void HAL_UART_MspInit1(UART_HandleTypeDef* huart) {
    GPIO_InitTypeDef GPIO_InitStruct;
    (void) GPIO_InitStruct;
#if (MODBUS_SER_PORT1_ENABLE > 0)

    if (huart->Instance == USART1)
    {
        /* Peripheral clock enable */
        __HAL_RCC_USART1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**USART2 GPIO Configuration
        PA9     ------> USART1_TX
        PA10     ------> USART1_RX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        /* USART1 interrupt Init */
        HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
    }

#endif
#if (MODBUS_SER_PORT2_ENABLE > 0)

    if (huart->Instance == USART2)
    {
        /* Peripheral clock enable */
        __HAL_RCC_USART2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**USART2 GPIO Configuration
        PA2     ------> USART2_TX
        PA3     ------> USART2_RX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        /* USART2 interrupt Init */
        HAL_NVIC_SetPriority(USART2_IRQn, 2, 0);
        HAL_NVIC_EnableIRQ(USART2_IRQn);
    }

#endif
#if (MODBUS_SER_PORT3_ENABLE > 0)

    if (huart->Instance == USART3) {
        /* Peripheral clock enable */
        __HAL_RCC_USART3_CLK_ENABLE();
        __HAL_RCC_GPIOD_CLK_ENABLE();
        /* USART3 GPIO Configuration
        PD8     ------> USART3_TX
        PD9     ------> USART3_RX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
        /* USART3 interrupt Init */
        HAL_NVIC_SetPriority(USART3_IRQn, 2, 0);
        HAL_NVIC_EnableIRQ(USART3_IRQn);
    }

#endif
#if (MODBUS_SER_PORT4_ENABLE > 0)

    if (huart->Instance == USART6)
    {
        /* Peripheral clock enable */
        __HAL_RCC_USART6_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();
        /**USART6 GPIO Configuration
        PC6     ------> USART6_TX
        PC7     ------> USART6_RX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
        HAL_NVIC_SetPriority(USART6_IRQn, 2, 0);
        HAL_NVIC_EnableIRQ(USART6_IRQn);
    }

#endif
}

void HAL_UART_MspDeInit1(UART_HandleTypeDef* huart) {
#if (MODBUS_SER_PORT1_ENABLE > 0)

    if (huart->Instance == USART1)
    {
        /* Peripheral clock disable */
        __HAL_RCC_USART1_CLK_DISABLE();
        /**USART1 GPIO Configuration
        PA9     ------> USART1_TX
        PA10     ------> USART1_RX
        */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9 | GPIO_PIN_10);
        /* USART2 interrupt DeInit */
        HAL_NVIC_DisableIRQ(USART1_IRQn);
    }

#endif
#if (MODBUS_SER_PORT2_ENABLE > 0)

    if (huart->Instance == USART2)
    {
        /* Peripheral clock disable */
        __HAL_RCC_USART2_CLK_DISABLE();
        /**USART2 GPIO Configuration
        PA2     ------> USART2_TX
        PA3     ------> USART2_RX
        */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2 | GPIO_PIN_3);
        /* USART2 interrupt DeInit */
        HAL_NVIC_DisableIRQ(USART2_IRQn);
    }

#endif
#if (MODBUS_SER_PORT3_ENABLE > 0)

    if (huart->Instance == USART3) {
        /* Peripheral clock disable */
        __HAL_RCC_USART3_CLK_DISABLE();
        /**USART2 GPIO Configuration
        PB10     ------> USART3_TX
        PB11     ------> USART3_RX
        */
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10 | GPIO_PIN_11);
        /* USART3 interrupt DeInit */
        HAL_NVIC_DisableIRQ(USART3_IRQn);
    }

#endif
#if (MODBUS_SER_PORT4_ENABLE > 0)

    if (huart->Instance == USART6)
    {
        /* Peripheral clock disable */
        __HAL_RCC_USART6_CLK_DISABLE();
        /**USART6 GPIO Configuration
        PC6     ------> USART6_TX
        PC7     ------> USART6_RX
        */
        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6 | GPIO_PIN_7);
    }

#endif
}

void mbport_serif_isr(mbport_serial_t* mb_serial);

#if (MODBUS_SER_PORT1_ENABLE > 0)
void USART1_IRQHandler(void)
{
    mbport_serif_isr(&mb_serial_1);
}
#endif

#if (MODBUS_SER_PORT2_ENABLE > 0)
void USART2_IRQHandler(void)
{
    mbport_serif_isr(&mb_serial_2);
}
#endif

#if (MODBUS_SER_PORT3_ENABLE > 0)

void USART3_IRQHandler(void) {
    mbport_serif_isr(&mb_serial_3);
}

#endif

#if (MODBUS_SER_PORT4_ENABLE > 0)
void USART6_IRQHandler(void)
{
    mbport_serif_isr(&mb_serial_4);
}
#endif

void mbport_serif_isr(mbport_serial_t* mb_serial) {
    /* UART in mode Transmitter */
    if (__HAL_UART_GET_IT_SOURCE(&mb_serial->handle, UART_IT_TXE) != RESET) {
    }

    /* UART in mode Receiver */
    if (__HAL_UART_GET_FLAG(&mb_serial->handle, USART_SR_RXNE) != RESET) {
        if (mb_serial->count < MOPORT_SER_ADU_SIZE_MAX) {
            mb_serial->buf[mb_serial->wr_pos] = (mb_uint8_t) mb_serial->handle.Instance->DR;
            mb_serial->wr_pos = (mb_serial->wr_pos + 1) % MOPORT_SER_ADU_SIZE_MAX;
            mb_serial->count += 1;
        }
    }
}

static mb_int32_t _mbport_serial_open(modbus_port_t* mb_port) {
    return MB_ENOERR;
}

static mb_int32_t _mbport_serial_close(modbus_port_t* mb_port) {
    return MB_ENOERR;
}

static mb_int32_t _mbport_serial_send(modbus_port_t* mb_port,
                                      mb_uint8_t* frame, mb_uint16_t length, mb_uint32_t timeout) {
    uint32_t flag;
    uint32_t tickstart;
    uint32_t remain_len;
    mbport_serial_t* mb_serial = (mbport_serial_t*) mb_port;
    remain_len = length;
    flag = UART_FLAG_TXE;
    tickstart = HAL_GetTick();
    MB_SER_RS485_PIN_RE(mb_serial, GPIO_PIN_SET);
    //MB_SER_RS485_PIN_DE(mb_serial, GPIO_PIN_SET);

    while (1) {
        if (remain_len == 0)
            flag = UART_FLAG_TC;

        /* Wait until flag is set */
        while ((__HAL_UART_GET_FLAG(&(mb_serial->handle), flag) ? SET : RESET) == RESET) {
            /* Check for the Timeout */
            if (timeout != HAL_MAX_DELAY) {
                if ((timeout == 0U) || ((HAL_GetTick() - tickstart) > timeout)) {
                    /* Disable TXE, RXNE, PE and ERR (Frame error, noise error,
                      overrun error) interrupts for the interrupt process */
                    CLEAR_BIT(mb_serial->handle.Instance->CR1,
                              (USART_CR1_RXNEIE | USART_CR1_PEIE | USART_CR1_TXEIE));
                    CLEAR_BIT(mb_serial->handle.Instance->CR3, USART_CR3_EIE);
                    goto err_exit;
                }
            }
        }

        if (remain_len == 0)
            break;

        mb_serial->handle.Instance->DR = (*frame++ & (uint8_t) 0xFF);
        remain_len -= 1;
    }

    err_exit:
    MB_SER_RS485_PIN_RE(mb_serial, GPIO_PIN_RESET);
    //MB_SER_RS485_PIN_DE(mb_serial, GPIO_PIN_RESET);
    return (length - remain_len);
}

static mb_int32_t _mbport_serial_recv(modbus_port_t* mb_port,
                                      mb_uint8_t* frame, mb_uint16_t length, mb_uint32_t timeout) {
    mb_uint8_t* data_ptr;
    uint32_t tickstart = 0U;
    mbport_serial_t* mb_serial = (mbport_serial_t*) mb_port;
    MB_SER_RS485_PIN_RE(mb_serial, GPIO_PIN_RESET);
    if (frame == NULL) {
        return -MB_ENORES;
    }

    data_ptr = frame;
    tickstart = HAL_GetTick();

    while (length > 0) {
        if (mb_serial->count != 0) {
            *data_ptr = mb_serial->buf[mb_serial->rd_pos];
            mb_serial->rd_pos = (mb_serial->rd_pos + 1) % MOPORT_SER_ADU_SIZE_MAX;
            mb_serial->count--;
            data_ptr += 1;
            length -= 1;
            timeout = mb_serial->ch_it;
            tickstart = HAL_GetTick();
        } else {
            /* no data in rx buffer */
            if ((HAL_GetTick() - tickstart) > timeout)
                break;
        }
    }

    return (mb_int32_t) (data_ptr - frame);
}

static mb_int32_t _mbport_serial_control(modbus_port_t* mb_port,
                                         mb_uint32_t cmd, void* args) {
    mbport_sercfg_t* sercfg = (mbport_sercfg_t*) args;
    mbport_serial_t* mb_serial = (mbport_serial_t*) mb_port;

    if (cmd == MODBUS_CTRL_READ) {
        /* BaudRate */
        sercfg->baud = mb_serial->handle.Init.BaudRate;

        /* Data Bits */
        if (mb_serial->handle.Init.WordLength == UART_WORDLENGTH_8B) {
            sercfg->data_bit = 8;
        } else {
            return -MB_EIO;
        }

        /* Parity */
        if (mb_serial->handle.Init.Parity == UART_PARITY_NONE) {
            sercfg->parity = 0;
        } else if (mb_serial->handle.Init.Parity == UART_PARITY_ODD) {
            sercfg->parity = 1;
        } else if (mb_serial->handle.Init.Parity == UART_PARITY_EVEN) {
            sercfg->parity = 2;
        } else {
            return -MB_EIO;
        }

        /* StopBit */
        if (mb_serial->handle.Init.StopBits == UART_STOPBITS_1) {
            sercfg->stop_bit = 1;
        } else if (mb_serial->handle.Init.StopBits == UART_STOPBITS_2) {
            sercfg->stop_bit = 2;
        } else {
            return -MB_EIO;
        }

        /* Other */
        sercfg->flow_ctl.cts = 0;

        if ((mb_serial->handle.Init.HwFlowCtl & UART_HWCONTROL_CTS) != 0) {
            sercfg->flow_ctl.cts = 1;
        }

        sercfg->flow_ctl.rts = 0;

        if ((mb_serial->handle.Init.HwFlowCtl & UART_HWCONTROL_RTS) != 0) {
            sercfg->flow_ctl.rts = 1;
        }
    } else if (cmd == MODBUS_CTRL_WRITE) {
        /* BaudRate */
        mb_serial->handle.Init.BaudRate = sercfg->baud;

        /* Data Bits */
        if (sercfg->data_bit == 8) {
            mb_serial->handle.Init.WordLength = UART_WORDLENGTH_8B;
        } else if (sercfg->data_bit == 9) {
            mb_serial->handle.Init.WordLength = UART_WORDLENGTH_9B;
        } else {
            return -MB_EIO;
        }

        /* Parity */
        if (sercfg->parity == 0) {
            mb_serial->handle.Init.Parity = UART_PARITY_NONE;
        } else if (sercfg->parity == 1) {
            mb_serial->handle.Init.Parity = UART_PARITY_ODD;
        } else if (sercfg->parity == 2) {
            mb_serial->handle.Init.Parity = UART_PARITY_EVEN;
        } else {
            return -MB_EIO;
        }

        /* StopBit */
        if (sercfg->stop_bit == 1) {
            mb_serial->handle.Init.StopBits = UART_STOPBITS_1;
        } else if (sercfg->stop_bit == 2) {
            mb_serial->handle.Init.StopBits = UART_STOPBITS_2;
        } else {
            return -MB_EIO;
        }

        /* Other */
        mb_serial->handle.Init.Mode = UART_MODE_TX_RX;
        mb_serial->handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;

        if (sercfg->flow_ctl.cts != 0) {
            mb_serial->handle.Init.HwFlowCtl |= UART_HWCONTROL_CTS;
        }

        if (sercfg->flow_ctl.rts != 0) {
            mb_serial->handle.Init.HwFlowCtl |= UART_HWCONTROL_RTS;
        }

        mb_serial->handle.Init.OverSampling = UART_OVERSAMPLING_16;
        mb_serial->ch_it = sercfg->ch_it;
    }

    return MB_ENOERR;
}


mbport_serial_t* mbport_serial_create(mb_uint8_t port_num) {
    mbport_serial_t* mb_serial = MB_NULL;
    mbport_sercfg_t mb_sercfg = SERIAL_CONFIG_DEFAULT;
#if (MODBUS_SER_PORT1_ENABLE > 0)

    if (port_num == 1)
    {
        mb_serial = &mb_serial_1;
        mb_serial->handle.Instance = USART1;
        MB_SER_CFG_RS485_PIN_RE(mb_serial, GPIOA, GPIO_PIN_6);
        MB_SER_CFG_RS485_PIN_DE(mb_serial, GPIOA, GPIO_PIN_7);
    }

#endif
#if (MODBUS_SER_PORT2_ENABLE > 0)

    if (port_num == 2)
    {
        mb_serial = &mb_serial_2;
        mb_serial->handle.Instance = USART2;
        MB_SER_CFG_RS485_PIN_RE(ser_if, GPIOA, GPIO_PIN_0);
        MB_SER_CFG_RS485_PIN_DE(ser_if, GPIOA, GPIO_PIN_1);
    }

#endif
#if (MODBUS_SER_PORT3_ENABLE > 0)

    if (port_num == 3) {
        mb_serial = &mb_serial_3;
        mb_serial->handle.Instance = USART3;
        MB_SER_CFG_RS485_PIN_RE(mb_serial, GPIOD, GPIO_PIN_10);
        MB_SER_CFG_RS485_PIN_DE(mb_serial, GPIOD, GPIO_PIN_10);
    }

#endif
#if (MODBUS_SER_PORT4_ENABLE > 0)

    if (port == 4)
    {
        mb_serial = &mb_serial_4;
        mb_serial->handle.Instance = USART6;
        MB_SER_CFG_RS485_PIN_RE(ser_if, GPIOB, GPIO_PIN_12);
        MB_SER_CFG_RS485_PIN_DE(ser_if, GPIOB, GPIO_PIN_13);
    }

#endif

    if (mb_serial == MB_NULL) {
        return MB_NULL;
    }

    _mbport_serial_control((modbus_port_t*) mb_serial, MODBUS_CTRL_WRITE,
                           &mb_sercfg);
    mb_serial->parent.open_fn = _mbport_serial_open;
    mb_serial->parent.close_fn = _mbport_serial_close;
    mb_serial->parent.send_fn = _mbport_serial_send;
    mb_serial->parent.recv_fn = _mbport_serial_recv;
    mb_serial->parent.ctrl_fn = _mbport_serial_control;
    return mb_serial;
}

#endif
