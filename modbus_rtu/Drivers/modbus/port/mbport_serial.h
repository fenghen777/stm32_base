/*
 * mbport_serial.h
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-01-25     nx           the first version
 *
 */

#ifndef __MBPORT_SERIAL_H_
#define __MBPORT_SERIAL_H_

#include "stm32f4xx_hal.h"
#include "modbus_cfg.h"
#include "modbus_priv.h"

#ifdef __cplusplus
extern "C" {
#endif

#if (MODBUS_SER_ENABLED > 0)

#define MOPORT_SER_ADU_SIZE_MAX             513

#define MODBUS_SER_PORT1_ENABLE             0
#define MODBUS_SER_PORT2_ENABLE             0
#define MODBUS_SER_PORT3_ENABLE             3
#define MODBUS_SER_PORT4_ENABLE             0

/* Default config for mbport_sercfg structure */
#define SERIAL_CONFIG_DEFAULT           \
    {                                   \
        9600,  /* 9600 bits/s */        \
        8,     /* 8 databits */         \
        0,     /* No parity  */         \
        1,     /* 1 stopbit */          \
        { 0, 0, 0 },                    \
        10                              \
    }

#define MBPORT_SER_CFG(cfg, br, db, pa, \
    sb, bcts, brts, chit)               \
    do {                                \
        (cfg)->baud = br;               \
        (cfg)->data_bit = db;           \
        (cfg)->parity = pa;             \
        (cfg)->stop_bit = sb;           \
        (cfg)->flow_ctl.zero = 0;       \
        (cfg)->flow_ctl.cts = bcts;     \
        (cfg)->flow_ctl.rts = brts;     \
        (cfg)->ch_it = chit;            \
    } while(0)

typedef struct mbport_sercfg
{
    mb_uint32_t baud;                  /* Bauds: 9600, 19200, 57600, 115200, etc */
    mb_uint8_t data_bit;               /* Data bit: 8, 9 */
    mb_uint8_t parity;                 /* Parity: 0 - NONE, 1 - ODD, 2 - EVEN */
    mb_uint8_t stop_bit;               /* Stop bit: 1, 2 */

    struct
    {
        mb_uint8_t zero     : 6;
        mb_uint8_t cts      : 1;
        mb_uint8_t rts      : 1;
    } flow_ctl;                         /* hardware flow control */

    mb_uint8_t ch_it;                   /* Character interval time */
} mbport_sercfg_t;



typedef struct mbport_serial
{
    modbus_port_t parent;

    uint8_t state;
    uint32_t ch_it;

    UART_HandleTypeDef handle;

    struct
    {
        GPIO_TypeDef *gpio_re;
        uint16_t pin_re;
        GPIO_TypeDef *gpio_de;
        uint16_t pin_de;
    } rs485_pin;

    /* Receive buffer */
    uint16_t count;
    uint16_t wr_pos;
    uint16_t rd_pos;
    uint8_t buf[MOPORT_SER_ADU_SIZE_MAX];

} mbport_serial_t;

mbport_serial_t *mbport_serial_create(mb_uint8_t port_num);

#endif

#ifdef __cplusplus
}
#endif

#endif

