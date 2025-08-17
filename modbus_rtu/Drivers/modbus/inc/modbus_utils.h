/*
 * modbus_utils.h
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-01-25     nx      	   the first version
 *
 */
 
#ifndef __MODBUS_UTILS_H_
#define __MODBUS_UTILS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "modbus_priv.h"

mb_uint8_t modbus_lrc(mb_uint8_t *frame, mb_uint16_t len);
mb_uint16_t modbus_crc16(mb_uint8_t *frame, mb_uint16_t len);

mb_uint8_t modbus_char_to_bin(mb_char_t character);
mb_char_t modbus_bin_to_char(mb_uint8_t byte);

void *modbus_memcpy(void *dst, const void *src, mb_uint32_t count);
void *modbus_memset(void *s, int c, mb_uint32_t count);
mb_size_t modbus_strlen(const char *s);

void modbus_set_bits_from_byte(mb_uint8_t *dest,
    mb_int32_t idx, const mb_uint8_t value);
void modbus_set_bits_from_bytes(mb_uint8_t *dest, mb_int32_t idx,
    mb_uint32_t nb_bits, const mb_uint8_t *tab_byte);
mb_uint8_t modbus_get_byte_from_bits(const mb_uint8_t *src,
    mb_int32_t idx, mb_uint32_t nb_bits);
mb_float_t modbus_get_float_abcd(const mb_uint16_t *src);
mb_float_t modbus_get_float_dcba(const mb_uint16_t *src);
mb_float_t modbus_get_float_badc(const mb_uint16_t *src);
mb_float_t modbus_get_float_cdab(const mb_uint16_t *src);
mb_float_t modbus_get_float(const mb_uint16_t *src);
void modbus_set_float_abcd(mb_float_t f, mb_uint16_t *dest);
void modbus_set_float_dcba(mb_float_t f, mb_uint16_t *dest);
void modbus_set_float_badc(mb_float_t f, mb_uint16_t *dest);
void modbus_set_float_cdab(mb_float_t f, mb_uint16_t *dest);
void modbus_set_float(mb_float_t f, mb_uint16_t *dest);

#ifdef __cplusplus
}
#endif

#endif
