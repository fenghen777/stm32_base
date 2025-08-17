/*
 * modbus_func.h
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-01-25     nx      	   the first version
 *
 */

#ifndef __MODBUS_FUNC_H_
#define __MODBUS_FUNC_H_

#include "modbus_priv.h"

#ifdef __cplusplus
extern "C" {
#endif

mb_exception_t mbfunc_read_coils(modbus_t *mb, mb_uint8_t *frame, mb_uint16_t *length);
mb_exception_t mbfunc_read_discrete_inputs(modbus_t *mb, mb_uint8_t *frame, mb_uint16_t *length);
mb_exception_t mbfunc_read_holding_registers(modbus_t *mb, mb_uint8_t *frame, mb_uint16_t *length);
mb_exception_t mbfunc_read_input_registers(modbus_t *mb, mb_uint8_t *frame, mb_uint16_t *length);
mb_exception_t mbfunc_write_single_coil(modbus_t *mb, mb_uint8_t *frame, mb_uint16_t *length);
mb_exception_t mbfunc_write_single_register(modbus_t *mb, mb_uint8_t *frame, mb_uint16_t *length);
mb_exception_t mbfunc_write_multiple_coils(modbus_t *mb, mb_uint8_t *frame, mb_uint16_t *length);
mb_exception_t mbfunc_write_multiple_registers(modbus_t *mb, mb_uint8_t *frame, mb_uint16_t *length);
mb_exception_t mbfunc_report_slave_id(modbus_t *mb, mb_uint8_t *frame, mb_uint16_t *length);
mb_exception_t mbfunc_mask_write_register(modbus_t *mb, mb_uint8_t *frame, mb_uint16_t *length);
mb_exception_t mbfunc_write_and_read_registers(modbus_t *mb, mb_uint8_t *frame, mb_uint16_t *length);

#ifdef __cplusplus
}
#endif
			


#endif
