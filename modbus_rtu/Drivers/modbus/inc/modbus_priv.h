/*
 * modbus_priv.h
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-01-25     nx           the first version
 *
 */
#ifndef __MODBUS_PRIV_H_
#define __MODBUS_PRIV_H_

#include "modbus_cfg.h"

#ifdef __cplusplus
extern "C" {
#endif


/* The full version, like 1.2.3 */
#define MODBUS_VERSION                      0.0.2

/* The full version, in string form (suited for string concatenation)
 */
#define MODBUS_VERSION_STRING               "0.0.2"


/* The size of the MODBUS PDU is limited by the size constraint inherited from
 * the first MODBUS implementation on Serial Line network (max. RS485 ADU = 256
 * bytes). Therefore, MODBUS PDU for serial line communication = 256 - Server
 * address (1 byte) - CRC (2 bytes) = 253 bytes.
 */
#define MODBUS_PDU_SIZE_MAX                 253         /** Maximum size of a PDU. */

/* Consequently:
 * - RTU MODBUS ADU = 253 bytes + Server address (1 byte) + CRC (2 bytes) = 256
 *   bytes.
 * - ASCII MODBUS ADU = Start ':' (1 byte) + 506 bytes + Server address (2 byte) +
 *   LRC (2 bytes) + End '\r\n' (2 byte) = 513 bytes.
 * - TCP MODBUS ADU = 253 bytes + MBAP (7 bytes) = 260 bytes.
 */
#define MODBUS_ADU_SIZE_MAX                 260

#define MODBUS_ADDR_BROADCAST               0   /** Modbus broadcast address. */
#define MODBUS_ADDR_MIN                     1   /** Smallest possible slave address. */
#define MODBUS_ADDR_MAX                     247 /** Biggest possible slave address. */

#define MODBUS_FC_NONE                      (0x00)
#define MODBUS_FC_READ_COILS                (0x01)
#define MODBUS_FC_READ_DISCRETE_INPUTS      (0x02)
#define MODBUS_FC_READ_HOLDING_REGISTERS    (0x03)
#define MODBUS_FC_READ_INPUT_REGISTERS      (0x04)
#define MODBUS_FC_WRITE_SINGLE_COIL         (0x05)
#define MODBUS_FC_WRITE_SINGLE_REGISTER     (0x06)
#define MODBUS_FC_READ_EXCEPTION_STATUS     (0x07)
#define MODBUS_FC_WRITE_MULTIPLE_COILS      (0x0F)
#define MODBUS_FC_WRITE_MULTIPLE_REGISTERS  (0x10)
#define MODBUS_FC_REPORT_SLAVE_ID           (0x11)
#define MODBUS_FC_MASK_WRITE_REGISTER       (0x16)
#define MODBUS_FC_WRITE_AND_READ_REGISTERS  (0x17)
#define MODBUS_FC_READ_DEVICE_ID            (0x2B)
#define MODBUS_FC_ERROR                     (0x80)


/* Modbus_Application_Protocol_V1_1b.pdf (chapter 6 section 1 page 12)
 * Quantity of Coils to read (2 bytes): 1 to 2000 (0x7D0)
 * (chapter 6 section 11 page 29)
 * Quantity of Coils to write (2 bytes): 1 to 1968 (0x7B0)
 */
#define MODBUS_MAX_READ_BITS                2000
#define MODBUS_MAX_WRITE_BITS               1968

/* Modbus_Application_Protocol_V1_1b.pdf (chapter 6 section 3 page 15)
 * Quantity of Registers to read (2 bytes): 1 to 125 (0x7D)
 * (chapter 6 section 12 page 31)
 * Quantity of Registers to write (2 bytes) 1 to 123 (0x7B)
 * (chapter 6 section 17 page 38)
 * Quantity of Registers to write in R/W registers (2 bytes) 1 to 121 (0x79)
 */
#define MODBUS_MAX_READ_REGISTERS           125
#define MODBUS_MAX_WRITE_REGISTERS          123
#define MODBUS_MAX_WR_WRITE_REGISTERS       121
#define MODBUS_MAX_WR_READ_REGISTERS        125

/* It's not really the minimal length (the real one is report slave ID
 * in RTU (4 bytes)) but it's a convenient size to use in RTU or TCP
 * communications to read many values or write a single one.
 * Maximum between :
 * - HEADER_LENGTH_TCP (7) + function (1) + address (2) + number (2)
 * - HEADER_LENGTH_RTU (1) + function (1) + address (2) + number (2) + CRC (2)
 */
#define MODBUS_MIN_REQ_LENGTH               12

#define MODBUS_REPORT_SLAVE_ID              180

#define MODBUS_MAGIC                        0x1A2B

/* modbus flag define */
#define MODBUS_FLAG_DISABLED                0x00
#define MODBUS_FLAG_ENABLED                 0x01

/* modbus mode define */
#define MODBUS_MODE_NONE                    0
#define MODBUS_MODE_ASCII                   1
#define MODBUS_MODE_RTU                     2


#define MODBUS_CTRL_RW_MASK                 (mb_uint32_t)(7UL << 29)
#define MODBUS_CTRL_READ                    (mb_uint32_t)(0UL << 29)
#define MODBUS_CTRL_WRITE                   (mb_uint32_t)(1UL << 29)
#define MODBUS_CTRL_LOCK                    (mb_uint32_t)(2UL << 29)

#if (MODBUS_USE_DEBUG != 0)

#include <stdio.h>

#define MODBUS_DBG(message, ...)            MOBUS_DEBUG_PRINT(message, ##__VA_ARGS__)
#else
#define MODBUS_DBG(message, ...)
#endif

#define MB_NULL                             0

/* Type define */
typedef signed         char     mb_int8_t;
typedef signed short   int      mb_int16_t;
typedef signed         int      mb_int32_t;
typedef unsigned       char     mb_uint8_t;
typedef unsigned short int      mb_uint16_t;
typedef unsigned       int      mb_uint32_t;
typedef char                    mb_char_t;
typedef float                   mb_float_t;
typedef unsigned       int      mb_size_t;
typedef signed         int      mb_err_t;


/* Protocol exceptions */
typedef enum mb_exception {
    MB_EX_NONE                  = 0x00,
    MB_EX_ILLEGAL_FUNCTION      = 0x01,
    MB_EX_ILLEGAL_DATA_ADDRESS  = 0x02,
    MB_EX_ILLEGAL_DATA_VALUE    = 0x03,
    MB_EX_SLAVE_DEVICE_FAILURE  = 0x04,
    MB_EX_ACKNOWLEDGE           = 0x05,
    MB_EX_SLAVE_BUSY            = 0x06,
    MB_EX_MEMORY_PARITY_ERROR   = 0x08,
    MB_EX_GATEWAY_PATH_FAILED   = 0x0A,
    MB_EX_GATEWAY_TGT_FAILED    = 0x0B,
    MB_EX_MAX                   = 0x0C,
} mb_exception_t;

/* Protocol errorcodes */
typedef enum mb_errcode {
    MB_ENOERR       = 0,
    MB_EILFUN       = 1,        /** Illegal function */
    MB_EILADD       = 2,        /** Illegal data address */
    MB_EILVAL       = 3,        /** Illegal data value */
    MB_ESFAIL       = 4,        /** Slave device or server failure */
    MB_EACK         = 5,        /** Acknowledge */
    MB_ESBUSY       = 6,        /** Slave device or server is busy */
    MB_ENACK        = 7,        /** Negative acknowledge */
    MB_EMEMPAR      = 8,        /** Memory parity error */
    MB_ENOTDEF      = 9,        /** Not defined */
    MB_EGPATH       = 10,       /** Gateway path unavailable */
    MB_EGTAR        = 11,       /** Target device failed to respond */
    MB_EBADCRC      = 12,       /** Invalid CRC */
    MB_EBADDATA     = 13,       /** Invalid data */
    MB_EBADEXC      = 14,       /** Invalid exception code */
    MB_EMDATA       = 15,       /** Too many data */
    MB_EBADSLAVE    = 16,       /** Response not from requested slave */
    MB_ENORES       = 17,       /** Insufficient resources. */
    MB_EIO          = 18,       /** I/O error. */
    MB_ETIMEDOUT    = 19,       /** Timeout error occurred. */
    MB_EILLSTATE    = 20,       /** Protocol stack in illegal state. */
    MB_EMAX
} mb_errcode_t;


/* Modbus register mapping */
typedef struct modbus_mapping {
    mb_uint16_t nb_bits;
    mb_uint16_t start_bits;
    mb_uint8_t* tab_bits;

    mb_uint16_t nb_input_bits;
    mb_uint16_t start_input_bits;
    mb_uint8_t* tab_input_bits;

    mb_uint16_t nb_input_registers;
    mb_uint16_t start_input_registers;
    mb_uint16_t* tab_input_registers;

    mb_uint16_t nb_registers;
    mb_uint16_t start_registers;
    mb_uint16_t* tab_registers;
} modbus_mapping_t;

/* Modbus port structure pre-defined */
typedef struct modbus_port modbus_port_t;

/* Modbus object structure */
typedef struct modbus {
    mb_uint16_t magic;
    mb_uint8_t state;                    /** Modbus state, enable or disable */
    mb_uint16_t tid;                     /** Modbus TID */

    mb_uint8_t mode;                     /** Modbus connection mode, ascii/rtu/any */
    mb_uint8_t slave_id;                 /** Slave ID */
    mb_int32_t rcv_tmo;                             /** Timeout for receiving data frames */

    mb_int32_t snd_buf_cnt;                         /** Send buffer counter  */
    mb_uint8_t snd_buf[MODBUS_PDU_SIZE_MAX + 7];    /** Send buffer pointer */

    mb_int32_t rcv_buf_cnt;                         /** Receive the current pointer to the cache  */
    mb_uint8_t rcv_buf[MODBUS_PDU_SIZE_MAX + 7];    /** Receive buffer pointer */

    modbus_mapping_t* map;                          /** Use only on the device side */

    modbus_port_t* port;
} modbus_t;

/* Modbus port structure */
struct modbus_port {
    mb_int32_t (*open_fn)(modbus_port_t* mb_port);

    mb_int32_t (*close_fn)(modbus_port_t* mb_port);

    mb_int32_t (*send_fn)(modbus_port_t* mb_port, mb_uint8_t* frame, mb_uint16_t length, mb_uint32_t timeout);

    mb_int32_t (*recv_fn)(modbus_port_t* mb_port, mb_uint8_t* frame, mb_uint16_t length, mb_uint32_t timeout);

    mb_int32_t (*ctrl_fn)(modbus_port_t* mb_port, mb_uint32_t cmd, void* args);
};


#ifdef __cplusplus
}
#endif

#endif

