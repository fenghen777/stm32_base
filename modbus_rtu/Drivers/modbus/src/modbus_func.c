/*
 * modbus_func.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-01-25     nx           the first version
 * 2023-03-21     nx           add conditional compilation
 *
 */

#include "modbus_func.h"
#include "modbus_utils.h"

#if ((MODBUS_DEVICE_MODE == 2) || (MODBUS_DEVICE_MODE == 3))
/**
  \fn             mb_exception_t modbus_read_coils(modbus_t *mb,
                      mb_uint8_t *frame, mb_uint16_t *length)
  \brief          Read coils
  \param[in]      mb        A pointer to Modbus objcet
  \param[in][out] frame     A pointer to PDU frame
  \param[in][out] length    A pointer to PDU length
  \return         Exception code
*/
#if (MB_FUNC_READ_COILS_ENABLED > 0)
mb_exception_t mbfunc_read_coils(modbus_t *mb, mb_uint8_t *frame, mb_uint16_t *length)
{
    mb_uint16_t i;
    mb_uint8_t shift = 0;
    mb_uint8_t one_byte = 0;

    mb_uint16_t address;
    mb_uint16_t coil_num;
    mb_uint16_t rsp_len;
    mb_int32_t mapping_addr;
    
    address = (frame[1] << 8) +
               frame[1 + 1];
               
    coil_num = (frame[1 + 2] << 8) +
                frame[1 + 3];
                
    mapping_addr = address - mb->map->start_bits;
    
    if ((coil_num < 1) || (MODBUS_MAX_READ_BITS < coil_num))
    {
        MODBUS_DBG("Illegal coil_num of values %d in read_coils (max %d)\n",
            coil_num, MODBUS_MAX_READ_BITS);
        return MB_EX_ILLEGAL_DATA_VALUE;
    }
    else if (mapping_addr < 0 || (mapping_addr + coil_num) > mb->map->nb_bits)
    {
        MODBUS_DBG("Illegal data address 0x%0X in read_coils\n",
            mapping_addr < 0 ? address : address + coil_num);
        return MB_EX_ILLEGAL_DATA_ADDRESS;
    }
    else
    {
        rsp_len = 0;
        frame[rsp_len++] = MODBUS_FC_READ_COILS;
        frame[rsp_len++] = (coil_num / 8) + ((coil_num % 8) ? 1 : 0);
        for (i = mapping_addr; i < mapping_addr + coil_num; i++)
        {
            one_byte |= mb->map->tab_bits[i] << shift;
            if (shift == 7)
            {
                /* Byte is full */
                frame[rsp_len++] = one_byte;
                one_byte = 0;
                shift = 0;
            }
            else
            {
                shift += 1;
            }
        }
        if (shift != 0)
            frame[rsp_len++] = one_byte;
    }
    *length = rsp_len;
    
    return MB_EX_NONE;
}
#endif

/**
  \fn             mb_exception_t mbfunc_read_discrete_inputs(modbus_t *mb,
                      mb_uint8_t *frame, mb_uint16_t *length)
  \brief          Read discrete inputs
  \param[in]      mb        A pointer to Modbus objcet
  \param[in][out] frame     A pointer to PDU frame
  \param[in][out] length    A pointer to PDU length
  \return         Exception code
*/
#if (MB_FUNC_READ_DISCRETE_INPUTS_ENABLED > 0)
mb_exception_t mbfunc_read_discrete_inputs(modbus_t *mb, mb_uint8_t *frame,
    mb_uint16_t *length)
{
    mb_uint16_t i;
    mb_uint8_t shift = 0;
    mb_uint8_t one_byte = 0;
    mb_uint16_t address;
    mb_uint16_t coil_num;
    mb_uint16_t rsp_len;
    mb_int32_t mapping_addr;
    
    address = (frame[1] << 8) +
               frame[1 + 1];
               
    coil_num = (frame[1 + 2] << 8) +
                frame[1 + 3];
                
    mapping_addr = address - mb->map->start_input_bits;
    if ((coil_num < 1) || (MODBUS_MAX_READ_BITS < coil_num))
    {
        MODBUS_DBG("Illegal coil_num of values %d in read_discrete_inputs (max %d)\n",
            coil_num, MODBUS_MAX_READ_BITS);
        return MB_EX_ILLEGAL_DATA_VALUE;
    }
    else if (mapping_addr < 0 || (mapping_addr + coil_num) > mb->map->nb_input_bits)
    {
        MODBUS_DBG("Illegal data address 0x%0X in read_discrete_inputs\n",
            mapping_addr < 0 ? address : address + coil_num);
        return MB_EX_ILLEGAL_DATA_ADDRESS;
    }
    else
    {
        rsp_len = 0;
        frame[rsp_len++] = MODBUS_FC_READ_DISCRETE_INPUTS;
        frame[rsp_len++] = (coil_num / 8) + ((coil_num % 8) ? 1 : 0);
        
        for (i = mapping_addr; i < mapping_addr + coil_num; i++)
        {
            one_byte |= mb->map->tab_input_bits[i] << shift;
            
            if (shift == 7)
            {
                /* Byte is full */
                frame[rsp_len++] = one_byte;
                one_byte = 0;
                shift = 0;
            }
            else
            {
                shift += 1;
            }
        }
        
        if (shift != 0)
            frame[rsp_len++] = one_byte;
    }
    
    *length = rsp_len;
    return MB_EX_NONE;
}
#endif

/**
  \fn             mb_exception_t mbfunc_read_holding_registers(modbus_t *mb,
                      mb_uint8_t *frame, mb_uint16_t *length)
  \brief          Read holding registers
  \param[in]      mb        A pointer to Modbus objcet
  \param[in][out] frame     A pointer to PDU frame
  \param[in][out] length    A pointer to PDU length
  \return         Exception code
*/
#if (MB_FUNC_READ_HOLDING_ENABLED > 0)
mb_exception_t mbfunc_read_holding_registers(modbus_t *mb, mb_uint8_t *frame,
    mb_uint16_t *length)
{
    mb_uint16_t i;
    mb_uint16_t address;
    mb_uint16_t reg_num;
    mb_uint16_t rsp_len;
    mb_int32_t mapping_addr;
    
    address = (frame[1] << 8) +
               frame[1 + 1];
               
    reg_num = (frame[1 + 2] << 8) +
               frame[1 + 3];
               
    mapping_addr = address - mb->map->start_registers;

    if (reg_num < 1 || MODBUS_MAX_READ_REGISTERS < reg_num)
    {
        MODBUS_DBG("Illegal reg_num of values %d in read_holding_registers (max %d)\n",
            reg_num, MODBUS_MAX_READ_REGISTERS);
        return MB_EX_ILLEGAL_DATA_VALUE;
    }
    else if (mapping_addr < 0 || (mapping_addr + reg_num) > mb->map->nb_registers)
    {
        MODBUS_DBG("Illegal data address 0x%0X in read_holding_registers\n",
            mapping_addr < 0 ? address : address + reg_num);
        return MB_EX_ILLEGAL_DATA_ADDRESS;
    }
    else
    {
        rsp_len = 0;
        frame[rsp_len++] = MODBUS_FC_READ_HOLDING_REGISTERS;
        frame[rsp_len++] = (mb_uint8_t)(reg_num * 2);
        for (i = mapping_addr; i < mapping_addr + reg_num; i++)
        {
            frame[rsp_len++] = mb->map->tab_registers[i] >> 8;
            frame[rsp_len++] = mb->map->tab_registers[i] & 0xFF;
        }
    }
    
    *length = rsp_len;
    return MB_EX_NONE;
}
#endif

/**
  \fn             mb_exception_t mbfunc_read_input_registers(modbus_t *mb,
                      mb_uint8_t *frame, mb_uint16_t *length)
  \brief          Read input registers
  \param[in]      mb        A pointer to Modbus objcet
  \param[in][out] frame     A pointer to PDU frame
  \param[in][out] length    A pointer to PDU length
  \return         Exception code
*/
#if (MB_FUNC_READ_INPUT_ENABLED > 0)
mb_exception_t mbfunc_read_input_registers(modbus_t *mb, mb_uint8_t *frame,
    mb_uint16_t *length)
{
    mb_uint16_t i;
    mb_uint16_t address;
    mb_uint16_t reg_num;
    mb_uint16_t rsp_len;
    mb_int32_t mapping_addr;
    address = (frame[1] << 8) +
               frame[1 + 1];
    reg_num = (frame[1 + 2] << 8) +
               frame[1 + 3];
    mapping_addr = address - mb->map->start_input_registers;

    if (reg_num < 1 || MODBUS_MAX_READ_REGISTERS < reg_num)
    {
        MODBUS_DBG("Illegal reg_num of values %d in read_input_registers (max %d)\n",
            reg_num, MODBUS_MAX_READ_REGISTERS);
        return MB_EX_ILLEGAL_DATA_VALUE;
    }
    else if (mapping_addr < 0 || (mapping_addr + reg_num) > mb->map->nb_input_registers)
    {
        MODBUS_DBG("Illegal data address 0x%0X in read_input_registers\n",
            mapping_addr < 0 ? address : address + reg_num);
        return MB_EX_ILLEGAL_DATA_ADDRESS;
    }
    else
    {
        rsp_len = 0;
        frame[rsp_len++] = MODBUS_FC_READ_INPUT_REGISTERS;
        frame[rsp_len++] = (mb_uint8_t)(reg_num * 2);
        
        for (i = mapping_addr; i < mapping_addr + reg_num; i++)
        {
            frame[rsp_len++] = mb->map->tab_input_registers[i] >> 8;
            frame[rsp_len++] = mb->map->tab_input_registers[i] & 0xFF;
        }
    }
    
    *length = rsp_len;
    return MB_EX_NONE;
}
#endif

/**
  \fn             mb_exception_t mbfunc_write_single_coil(modbus_t *mb,
                      mb_uint8_t *frame, mb_uint16_t *length)
  \brief          Write single coil
  \param[in]      mb        A pointer to Modbus objcet
  \param[in][out] frame     A pointer to PDU frame
  \param[in][out] length    A pointer to PDU length
  \return         Exception code
*/
#if (MB_FUNC_WRITE_COIL_ENABLED > 0)
mb_exception_t mbfunc_write_single_coil(modbus_t *mb, mb_uint8_t *frame,
    mb_uint16_t *length)
{
    mb_uint16_t address;
    mb_uint16_t reg_data;
    mb_int32_t mapping_addr;
    
    address = (frame[1] << 8) +
               frame[1 + 1];
               
    mapping_addr = address - mb->map->start_bits;
    
    if (mapping_addr < 0 || mapping_addr >= mb->map->nb_bits)
    {
        MODBUS_DBG("Illegal data address 0x%0X in write_bit\n",
            address);
        return MB_EX_ILLEGAL_DATA_ADDRESS;
    }
    else
    {
        reg_data = (frame[1 + 2] << 8) +
                    frame[1 + 3];
                    
        if (reg_data == 0xFF00 || reg_data == 0x0)
        {
            mb->map->tab_bits[mapping_addr] = reg_data ? 1 : 0;
        }
        else
        {
            MODBUS_DBG("Illegal data value 0x%0X in write_bit request at address %0X\n",
                reg_data, address);
            return MB_EX_ILLEGAL_DATA_VALUE;
        }
    }
    
    return MB_EX_NONE;
}
#endif

/**
  \fn             mb_exception_t mbfunc_write_single_register(modbus_t *mb,
                      mb_uint8_t *frame, mb_uint16_t *length)
  \brief          Write single register
  \param[in]      mb        A pointer to Modbus objcet
  \param[in][out] frame     A pointer to PDU frame
  \param[in][out] length    A pointer to PDU length
  \return         Exception code
*/
#if (MB_FUNC_WRITE_HOLDING_ENABLED > 0)
mb_exception_t mbfunc_write_single_register(modbus_t *mb, mb_uint8_t *frame,
    mb_uint16_t *length)
{
    mb_uint16_t address;
    mb_uint16_t reg_data;
    mb_int32_t mapping_addr;
    
    address = (frame[1] << 8) +
               frame[1 + 1];
               
    mapping_addr = address - mb->map->start_registers;
    if (mapping_addr < 0 || mapping_addr > mb->map->nb_registers)
    {
        MODBUS_DBG("Illegal data address 0x%0X in write_registers\n",
            address);
        return MB_EX_ILLEGAL_DATA_ADDRESS;
    }
    else
    {
        reg_data = (frame[1 + 2] << 8) +
                    frame[1 + 3];
        mb->map->tab_registers[mapping_addr] = reg_data;
    }
    return MB_EX_NONE;
}
#endif

/**
  \fn             mb_exception_t mbfunc_write_multiple_coils(modbus_t *mb,
                      mb_uint8_t *frame, mb_uint16_t *length)
  \brief          Write multiple coils
  \param[in]      mb        A pointer to Modbus objcet
  \param[in][out] frame     A pointer to PDU frame
  \param[in][out] length    A pointer to PDU length
  \return         Exception code
*/
#if (MB_FUNC_WRITE_MULTIPLE_COILS_ENABLED > 0)
mb_exception_t mbfunc_write_multiple_coils(modbus_t *mb, mb_uint8_t *frame,
    mb_uint16_t *length)
{
    mb_uint16_t address;
    mb_uint16_t coil_num;
    mb_int32_t mapping_addr;
    
    address = (frame[1] << 8) +
               frame[1 + 1];
               
    coil_num = (frame[1 + 2] << 8) +
                frame[1 + 3];
                
    mapping_addr = address - mb->map->start_bits;
    if (coil_num < 1 || MODBUS_MAX_WRITE_BITS < coil_num)
    {
        MODBUS_DBG("Illegal number of values %d in write_multiple_coils (max %d)\n",
                coil_num, MODBUS_MAX_WRITE_BITS);
        return MB_EX_ILLEGAL_DATA_VALUE;
    }
    else if (mapping_addr < 0 || (mapping_addr + coil_num) > mb->map->nb_bits)
    {
        MODBUS_DBG("Illegal data address 0x%0X in write_multiple_coils\n",
                mapping_addr < 0 ? address : address + coil_num);
        return MB_EX_ILLEGAL_DATA_ADDRESS;
    }
    else
    {
        modbus_set_bits_from_bytes(mb->map->tab_bits, mapping_addr, coil_num,
            &frame[1 + 5]);
        *length = 5;
    }
    return MB_EX_NONE;
}
#endif

/**
  \fn             mb_exception_t mbfunc_write_multiple_registers(modbus_t *mb,
                      mb_uint8_t *frame, mb_uint16_t *length)
  \brief          Write multiple registers
  \param[in]      mb        A pointer to Modbus objcet
  \param[in][out] frame     A pointer to PDU frame
  \param[in][out] length    A pointer to PDU length
  \return         Exception code
*/
#if (MB_FUNC_WRITE_MULTIPLE_HOLDING_ENABLED > 0)
mb_exception_t mbfunc_write_multiple_registers(modbus_t *mb, mb_uint8_t *frame,
    mb_uint16_t *length)
{
    mb_uint16_t i, j;
    mb_uint16_t address;
    mb_uint16_t reg_num;
    mb_int32_t mapping_addr;
    address = (frame[1] << 8) +
               frame[1 + 1];
               
    reg_num = (frame[1 + 2] << 8) +
               frame[1 + 3];
               
    mapping_addr = address - mb->map->start_registers;
    if (reg_num < 1 || MODBUS_MAX_WRITE_REGISTERS < reg_num)
    {
        MODBUS_DBG("Illegal number of values %d in write_multiple_registers (max %d)\n",
                reg_num, MODBUS_MAX_WRITE_REGISTERS);
        return MB_EX_ILLEGAL_DATA_VALUE;
    }
    else if (mapping_addr < 0 || (mapping_addr + reg_num) > mb->map->nb_registers)
    {
        MODBUS_DBG("Illegal data address 0x%0X in write_multiple_registers\n",
                mapping_addr < 0 ? address : address + reg_num);
        return MB_EX_ILLEGAL_DATA_ADDRESS;
    }
    else
    {
        for (i = mapping_addr, j = 5; i < mapping_addr + reg_num; i++, j += 2)
        {
            /* 5 and 6 = first value */
            mb->map->tab_registers[i] = (frame[1 + j] << 8) +
                                         frame[1 + j + 1];
        }
        *length = 5;
    }
    return MB_EX_NONE;
}
#endif

/**
  \fn             mb_exception_t mbfunc_report_slave_id(modbus_t *mb,
                      mb_uint8_t *frame, mb_uint16_t *length)
  \brief          Report slave id
  \param[in]      mb        A pointer to Modbus objcet
  \param[in][out] frame     A pointer to PDU frame
  \param[in][out] length    A pointer to PDU length
  \return         Exception code
*/
#if (MB_FUNC_REP_SLAVEID_ENABLED > 0)
mb_exception_t mbfunc_report_slave_id(modbus_t *mb, mb_uint8_t *frame,
    mb_uint16_t *length)
{
    mb_uint16_t str_len = 0;
    mb_uint16_t rsp_len = 0;
    
    frame[rsp_len++] = MODBUS_FC_REPORT_SLAVE_ID;
    /* Skip byte count for now */
    rsp_len += 1;
    /* SLAVE ID */
    frame[rsp_len++] = MODBUS_REPORT_SLAVE_ID;
    /* Run indicator status to ON */
    frame[rsp_len++] = 0xFF;
    /* MB  + length of MODBUS_VERSION_STRING */
    str_len = 2 + modbus_strlen(MODBUS_VERSION_STRING);
    modbus_memcpy(&frame[rsp_len], "MB" MODBUS_VERSION_STRING, str_len);
    rsp_len += str_len;
    frame[1] = rsp_len - 2;
    *length = rsp_len;
    return MB_EX_NONE;
}
#endif

/**
  \fn             mb_exception_t mbfunc_mask_write_register(modbus_t *mb,
                      mb_uint8_t *frame, mb_uint16_t *length)
  \brief          Mask write registers
  \param[in]      mb        A pointer to Modbus objcet
  \param[in][out] frame     A pointer to PDU frame
  \param[in][out] length    A pointer to PDU length
  \return         Exception code
*/
#if (MB_FUNC_MASK_WRITE_ENABLED > 0)
mb_exception_t mbfunc_mask_write_register(modbus_t *mb, mb_uint8_t *frame,
    mb_uint16_t *length)
{
    mb_uint16_t or_val;
    mb_uint16_t and_val;
    mb_uint16_t data_val;
    mb_uint16_t address;
    mb_int32_t mapping_addr;
    address = (frame[1] << 8) +
               frame[1 + 1];
               
    mapping_addr = address - mb->map->start_registers;
    
    if (mapping_addr < 0 || mapping_addr >= mb->map->nb_registers)
    {
        MODBUS_DBG("Illegal data address 0x%0X in write_registers\n",
            address);
        return MB_EX_ILLEGAL_DATA_ADDRESS;
    }
    else
    {
        data_val = mb->map->tab_bits[mapping_addr];
        and_val = (frame[1 + 2] << 8) +
                   frame[1 + 3];
        or_val = (frame[1 + 4] << 8) +
                  frame[1 + 5];
        data_val = (data_val & and_val) | (or_val & (~and_val));
        mb->map->tab_registers[mapping_addr] = data_val;
    }
    return MB_EX_NONE;
}
#endif

/**
  \fn             mb_exception_t mbfunc_write_and_read_registers(modbus_t *mb,
                      mb_uint8_t *frame, mb_uint16_t *length)
  \brief          Write and read registers
  \param[in]      mb        A pointer to Modbus objcet
  \param[in][out] frame     A pointer to PDU frame
  \param[in][out] length    A pointer to PDU length
  \return         Exception code
*/
#if (MB_FUNC_READWRITE_HOLDING_ENABLED > 0)
mb_exception_t mbfunc_write_and_read_registers(modbus_t *mb, mb_uint8_t *frame,
    mb_uint16_t *length)
{
    mb_uint16_t i, j;
    mb_uint16_t rsp_len;
    mb_uint16_t address_rd;
    mb_uint16_t reg_num_rd;
    mb_int32_t mapping_addr_rd;
    mb_uint16_t address_wr;
    mb_uint16_t reg_num_wr;
    mb_int32_t mapping_addr_wr;
    mb_uint8_t reg_wr_bytes;
    
    address_rd = (frame[1] << 8) +
                  frame[1 + 1];
    reg_num_rd = (frame[1 + 2] << 8) +
                  frame[1 + 3];
    address_wr = (frame[1 + 4] << 8) +
                  frame[1 + 5];
    reg_num_wr = (frame[1 + 6] << 8) +
                  frame[1 + 7];
    reg_wr_bytes = frame[1 + 8];
    
    mapping_addr_rd = address_rd - mb->map->start_registers;
    mapping_addr_wr = address_wr - mb->map->start_registers;
    if (reg_num_rd < 1 || MODBUS_MAX_READ_REGISTERS < reg_num_rd ||
        reg_num_wr < 1 || MODBUS_MAX_WRITE_REGISTERS < reg_num_wr ||
        reg_wr_bytes != reg_num_wr * 2)
    {
        MODBUS_DBG("Illegal reg_num of values (W%d, R%d) in write_and_read_registers (max W%d, R%d)\n",
                reg_num_wr, reg_num_rd, MODBUS_MAX_WRITE_REGISTERS, MODBUS_MAX_READ_REGISTERS);
        return MB_EX_ILLEGAL_DATA_VALUE;
    }
    else if (mapping_addr_rd < 0
        || (mapping_addr_rd + reg_num_rd) > mb->map->nb_registers ||
        mapping_addr_wr < 0 || (mapping_addr_wr + reg_num_wr) > mb->map->nb_registers)
    {
        MODBUS_DBG("Illegal data read address 0x%0X or write address 0x%0X write_and_read_registers\n",
                mapping_addr_rd < 0 ? address_rd : address_rd + reg_num_rd,
                mapping_addr_wr < 0 ? address_wr : address_wr + reg_num_wr);
        return MB_EX_ILLEGAL_DATA_ADDRESS;
    }
    else
    {
        /* Write first. */
        for (i = mapping_addr_wr, j = 9; i < mapping_addr_wr + reg_num_wr; i++, j += 2)
        {
            /* 5 and 6 = first value */
            mb->map->tab_registers[i] = (frame[1 + j] << 8) +
                                         frame[1 + j + 1];
        }
        rsp_len = 0;
        frame[rsp_len++] = MODBUS_FC_WRITE_AND_READ_REGISTERS;
        frame[rsp_len++] = reg_num_rd * 2;
        /* Read the data for the response */
        for (i = mapping_addr_rd; i < mapping_addr_rd + reg_num_rd; i++)
        {
            frame[rsp_len++] = mb->map->tab_registers[i] >> 8;
            frame[rsp_len++] = mb->map->tab_registers[i] & 0xFF;
        }
        *length = rsp_len;
    }
    return MB_EX_NONE;
}
#endif
#endif  // #if ((MODBUS_DEVICE_MODE == 2) || (MODBUS_DEVICE_MODE == 3))
