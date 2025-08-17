/*
 * modbus_api.h
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-01-25     nx      	   the first version
 *
 */

#ifndef __MODBUS_H_
#define __MODBUS_H_

#include "modbus_cfg.h"
#include "modbus_priv.h"
#include "modbus_utils.h"
#include "modbus_func.h"

#ifdef __cplusplus
extern "C" {
#endif


/** The following API is generic  */
modbus_t *modbus_create(mb_uint8_t conn_mode, void *port);

mb_err_t modbus_destroy(modbus_t *mb);

mb_err_t modbus_connect(modbus_t *mb);

mb_err_t modbus_disconn(modbus_t *mb);

mb_err_t modbus_control(modbus_t *mb, mb_uint32_t cmd, void *args);

mb_err_t modbus_send(modbus_t *mb, mb_uint8_t slave_id,
    const mb_uint8_t *frame, mb_uint16_t length);
    
mb_err_t modbus_recv(modbus_t *mb, mb_uint8_t *slave_id,
    mb_uint8_t **pframe, mb_uint16_t *length);
    
mb_err_t modbus_set_slave(modbus_t *mb, mb_uint8_t slave_id);

mb_err_t modbus_set_recvtmo(modbus_t *mb, mb_uint32_t time_ms);

/** The following API are used for master mode */
/* Master read */
mb_err_t modbus_read_bits(modbus_t *mb, 
    mb_uint16_t reg_addr, mb_uint16_t ncoils, mb_uint8_t *rsp);
    
mb_err_t modbus_read_input_bits(modbus_t *mb, 
    mb_uint16_t reg_addr, mb_uint16_t ncoils, mb_uint8_t *rsp);
    
mb_err_t modbus_read_holding_regs(modbus_t *mb,
    mb_uint16_t reg_addr, mb_uint16_t nregs, mb_uint16_t *rsp);
    
mb_err_t modbus_read_input_regs(modbus_t *mb,
    mb_uint16_t reg_addr, mb_uint16_t nregs, mb_uint16_t *rsp);

/* Master write */
mb_err_t modbus_write_coil(modbus_t *mb,
    mb_uint16_t reg_addr, mb_uint8_t status);
    
mb_err_t modbus_write_register(modbus_t *mb,
    mb_uint16_t reg_addr, mb_uint16_t value);
    
mb_err_t modbus_write_coils(modbus_t *mb,
    mb_uint16_t reg_addr, mb_uint16_t nregs, const mb_uint8_t *req);
    
mb_err_t modbus_write_registers(modbus_t *mb,
    mb_uint16_t reg_addr, mb_uint16_t nregs, const mb_uint16_t *req);

mb_err_t modbus_mask_write_register(modbus_t *mb, 
    mb_uint16_t reg_addr, mb_uint16_t and_mask, mb_uint16_t or_mask);
    
/* Master write and read */ 
mb_err_t modbus_write_and_read_registers(modbus_t *mb,
    mb_uint16_t read_addr, mb_uint16_t read_nb, mb_uint16_t *rsp,
    mb_uint16_t write_addr, mb_uint16_t write_nb, const mb_uint8_t *req);

/* Ohter */
mb_err_t modbus_report_slave_id(modbus_t *mb, 
    mb_uint16_t max_rsp, mb_uint8_t *rsp);
    
/** The following API are used for slave mode */
mb_err_t modbus_set_mapping(modbus_t *mb, modbus_mapping_t *map);

mb_err_t modbus_replay(modbus_t *mb,  mb_uint8_t slave_id,
    mb_uint8_t *frame, mb_uint16_t length);

const char *modbus_strerr(mb_err_t errcode);

#ifdef __cplusplus
}
#endif
																							 
#endif

