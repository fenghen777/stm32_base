/*
 * modbus_api.h
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-01-25     nx           the first version
 *
 */

#include "modbus_api.h"

static modbus_t modbus_table[MODBUS_OBJECT_NUM];
/*
 *  <---------------- MODBUS ADU(1) --------------------->
 *         <--------- MODBUS PDU (1') ----------->
 *  +------+-------------------------------------+-------+
 *  | ADD  | Code | Data                         | Check |
 *  +------+------+------------------------------+-------+
 *  |      |      |                              |
 * (2)    (3)    (4)                            (5)
 *
 * (1)  ... Modbus TCP/UDP Application Data Unit
 * (1') ... Modbus Protocol Data Unit
 * (2)  ... Modubs device address
 * (3)  ... Modubs function address
 * (4)  ... Modubs data uint
 * (5)  ... Modubs check code
 */

#if (MODBUS_ASCII_ENABLED > 0)
/**
  \brief  Send a data frame in Modbus ASCII mode
*/
static mb_err_t _mb_ascii_send(modbus_t *mb, mb_uint8_t slave_id,
    const mb_uint8_t *frame, mb_uint16_t length)
{
    mb_err_t ret;
    mb_uint8_t ch;
    mb_uint8_t pos = 0;
    mb_uint16_t idx = 0;
    mb_uint16_t checksum = 0;

    /* First byte before the Modbus-PDU is the slave address. */
    mb->snd_buf[6] = slave_id;
    mb->snd_buf_cnt = 1;

    /* Now copy the Modbus-PDU into the send buffer. */
    modbus_memcpy(&mb->snd_buf[7], frame, length);
    mb->snd_buf_cnt += length;

    /* Calculate LRC checksum for Modbus PDU. */
    checksum = modbus_lrc(&mb->snd_buf[6], mb->snd_buf_cnt);
    mb->snd_buf[6 + mb->snd_buf_cnt++] = checksum & 0xFF;

    /* The start of a frame is defined by sending the character ':'. */
    ch = ':';
    if (mb->port->send_fn(mb->port, &ch, 1, 100) != 1)
    {
        ret = -MB_EIO;
        goto err_exit;
    }
    /* Send the data block. Each data byte is encoded as a character hex
        stream with the high nibble sent first and the low nibble sent last. */
    for (idx = 0; idx < mb->snd_buf_cnt;)
    {
        if (pos == 0)
        {
            ch = modbus_bin_to_char((mb_uint8_t)(mb->snd_buf[6 + idx] >> 4));
            pos = 1;
        }
        else if (pos == 1)
        {
            ch = modbus_bin_to_char((mb_uint8_t)(mb->snd_buf[6 + idx] & 0x0F));
            pos = 0;
            idx += 1;
        }
        if (mb->port->send_fn(mb->port, &ch, 1, 100) != 1)
        {
            ret = -MB_EIO;
            goto err_exit;
        }
    }

    /* If all data bytes are exhausted we send a '\r' character
       to end the transmission. */
    if (mb->port->send_fn(mb->port, (mb_uint8_t *)("\r\n"), 2, 200) != 2)
    {
        ret = -MB_EIO;
        goto err_exit;
    }
err_exit:
    return ret;
}

/**
  \brief  Receive a data frame in Modbus ASCII mode
*/
static mb_err_t _mb_ascii_recv(modbus_t *mb, mb_uint8_t *slave_id,
    mb_uint8_t **pframe, mb_uint16_t *length)
{
    mb_err_t ret;
    mb_uint8_t ch;
    mb_uint8_t pos;
    mb_uint8_t step;
    mb_uint32_t timeout;

    pos = 0;
    step = 1;
    timeout = mb->rcv_tmo;
    while (1)
    {
        if (mb->port->recv_fn(mb->port, &ch, 1, timeout) <= 0)
        {
            ret = -MB_EIO;
            goto err_exit;
        }

        switch (step)
        {
        case 1:
            if (ch == ':')
            {
                step = 2;
                timeout = 100;
                mb->rcv_buf_cnt = 0;
            }
            break;
        case 2:
            if (ch == '\r')
            {
                step = 4;
            }
            else
            {
                ch = modbus_char_to_bin(ch);
                if (pos == 0)
                {
                    mb->rcv_buf[6 + mb->rcv_buf_cnt] = (mb_uint8_t)(ch << 4);
                    pos = 1;
                }
                else
                {
                    mb->rcv_buf[6 + mb->rcv_buf_cnt] |= (mb_uint8_t)ch;
                    mb->rcv_buf_cnt += 1;
                    pos = 0;
                }
            }
            break;
        case 4:
            if (ch == '\n')
            {
                if ((mb->rcv_buf_cnt >= 3) &&
                    (modbus_lrc(&mb->rcv_buf[6], mb->rcv_buf_cnt) == 0))
                {
                    *slave_id = mb->rcv_buf[6];
                    *pframe = &mb->rcv_buf[7];
                    *length = mb->rcv_buf_cnt - 2;
                    ret = MB_ENOERR;
                }
                else
                {
                    ret = -MB_EBADDATA;
                }
                goto err_exit;
            }
            break;
        }
    }

err_exit:
    return ret;
}
#endif

#if (MODBUS_RTU_ENABLED > 0)

/**
  \brief  Send a data frame in Modbus RTU mode
*/
static mb_err_t _mb_rtu_send(modbus_t* mb, mb_uint8_t slave_id,
                             const mb_uint8_t* frame, mb_uint16_t length) {
    mb_err_t ret;
    mb_uint16_t checksum = 0;

    /* First byte before the Modbus-PDU is the slave address. */
    mb->snd_buf[6] = slave_id;
    mb->snd_buf_cnt = 1;

    /* Now copy the Modbus-PDU into the send buffer. */
    modbus_memcpy(&mb->snd_buf[7], frame, length);
    mb->snd_buf_cnt += length;

    /* Calculate CRC16 checksum for Modbus PDU. */
    checksum = modbus_crc16(&mb->snd_buf[6], mb->snd_buf_cnt);
    mb->snd_buf[6 + mb->snd_buf_cnt++] = (mb_uint8_t) (checksum >> 8);
    mb->snd_buf[6 + mb->snd_buf_cnt++] = (mb_uint8_t) (checksum & 0xFF);

    if (mb->port->send_fn(mb->port, &mb->snd_buf[6],
                          mb->snd_buf_cnt, 3000) == mb->snd_buf_cnt) {
        ret = MB_ENOERR;
    } else {
        ret = -MB_EIO;
    }
    return ret;
}

/**
  \brief  Receive a data frame in Modbus RTU mode
*/
static mb_err_t _mb_rtu_recv(modbus_t* mb, mb_uint8_t* slave_id,
                             mb_uint8_t** pframe, mb_uint16_t* length) {
    mb_err_t ret;

    mb->rcv_buf_cnt = MODBUS_ADU_SIZE_MAX - 6;
    mb->rcv_buf_cnt = mb->port->recv_fn(mb->port,
                                        &mb->rcv_buf[6], mb->rcv_buf_cnt, mb->rcv_tmo);
    if (mb->rcv_buf_cnt == 0) {
        ret = -MB_ETIMEDOUT;
    } else if ((mb->rcv_buf_cnt >= 4) &&
               (modbus_crc16(&mb->rcv_buf[6], mb->rcv_buf_cnt) == 0)) {
        *slave_id = mb->rcv_buf[6];
        *pframe = &mb->rcv_buf[7];
        *length = mb->rcv_buf_cnt - 3;
        ret = MB_ENOERR;
    } else {
        ret = -MB_EBADDATA;
    }

    return ret;
}

#endif
/*
 *  <----------------------- MODBUS TCP/UDP ADU(1) ------------------------>
 *              <----------- MODBUS PDU (1') ---------------->
 *  +-----------+---------------+------------------------------------------+
 *  | TID | PID | Length | UID  |Code | Data                               |
 *  +-----------+---------------+------------------------------------------+
 *  |     |     |        |      |
 * (2)   (3)   (4)      (5)    (6)
 *
 * (2)  ... MB_TID          = 0 (Transaction Identifier - 2 Byte)
 * (3)  ... MB_PID          = 2 (Protocol Identifier - 2 Byte)
 * (4)  ... MB_LEN          = 4 (Number of bytes - 2 Byte)
 * (5)  ... MB_UID          = 6 (Unit Identifier - 1 Byte)
 * (6)  ... MB_FUNC         = 7 (Modbus Function Code)
 *
 * (1)  ... Modbus TCP/UDP Application Data Unit
 * (1') ... Modbus Protocol Data Unit
 */

#if (MODBUS_TCPUDP_ENABLED > 0)
#define MB_PROTOCOL_ID      0   /* 0 = Modbus Protocol */
/**
  \brief  Send a data frame in Modbus TCP/UDP mode
*/
static mb_err_t _mb_net_send(modbus_t *mb, mb_uint8_t slave_id,
    const mb_uint8_t *frame, mb_uint16_t length)
{
    mb_err_t ret;
    mb_int32_t snd_len;

    mb->snd_buf[0] = mb->tid >> 8U;
    mb->snd_buf[1] = mb->tid & 0xFF;
    mb->snd_buf[2] = MB_PROTOCOL_ID >> 8U;
    mb->snd_buf[3] = MB_PROTOCOL_ID & 0xFF;
    mb->snd_buf[4] = (length + 1) >> 8U;
    mb->snd_buf[5] = (length + 1) & 0xFF;
    mb->snd_buf[6] = slave_id;
    modbus_memcpy(&mb->snd_buf[7], frame, length);

    mb->snd_buf_cnt = 7 + length;
    snd_len = mb->port->send_fn(mb->port, mb->snd_buf, mb->snd_buf_cnt, 1000);
    if (snd_len == mb->snd_buf_cnt)
    {
        ret = MB_ENOERR;
    }
    else
    {
        ret = -MB_EIO;
    }

    return ret;
}

/**
  \brief  Receive a data frame in Modbus TCP/UDP mode
*/
static mb_err_t _mb_net_recv(modbus_t *mb, mb_uint8_t *slave_id,
    mb_uint8_t **pframe, mb_uint16_t *length)
{
    mb_err_t ret;
    mb_uint16_t pid;

    mb->rcv_buf_cnt = mb->port->recv_fn(mb->port,
            &mb->rcv_buf[0], MODBUS_ADU_SIZE_MAX, mb->rcv_tmo);

    if (mb->rcv_buf_cnt <= 0)
    {
        ret = -MB_EIO;
    }
    else
    {
        mb->tid = mb->rcv_buf[0] << 8U;
        mb->tid |= mb->rcv_buf[1];
        pid = mb->rcv_buf[2] << 8U;
        pid |= mb->rcv_buf[3];

        if (pid == MB_PROTOCOL_ID)
        {
            *slave_id = mb->rcv_buf[6];
            *pframe = &mb->rcv_buf[7];
            *length = mb->rcv_buf_cnt - 7;
            ret = MB_ENOERR;
        }
        else
        {
            ret = -MB_EIO;
        }
    }

    return ret;
}

#endif

#if ((MODBUS_DEVICE_MODE == 1) || (MODBUS_DEVICE_MODE == 3))

/**
  \brief  Computes the length of the expected response
*/
static mb_int16_t compute_rsp_len_from_req(mb_uint8_t* req) {
    mb_int16_t length;

    switch (req[0]) {
        case MODBUS_FC_READ_COILS:
        case MODBUS_FC_READ_DISCRETE_INPUTS: {
            mb_uint16_t byte_count;
            /* Header + nb values (code from write_bits) */
            byte_count = (req[3] << 8) | req[4];
            length = 2 + (byte_count / 8) + ((byte_count % 8) ? 1 : 0);
        }
            break;

        case MODBUS_FC_WRITE_AND_READ_REGISTERS:
        case MODBUS_FC_READ_HOLDING_REGISTERS:
        case MODBUS_FC_READ_INPUT_REGISTERS:
            /* Header + 2 * nb values */
            length = 2 + 2 * (req[3] << 8 | req[4]);
            break;

        case MODBUS_FC_READ_EXCEPTION_STATUS:
            length = 3;
            break;

        case MODBUS_FC_REPORT_SLAVE_ID:
            /* The response is device specific (the header provides the
               length) */
            length = -1;
            break;

        case MODBUS_FC_MASK_WRITE_REGISTER:
            length = 7;
            break;

        default:
            length = 5;
    }

    return length;
}

/**
  \brief  Check confirmation frame
*/
static mb_err_t _mb_check_confirm(modbus_t* mb, mb_uint8_t* req,
                                  mb_uint8_t slave_id, mb_uint8_t* rsp, mb_uint16_t rsp_len) {
    mb_err_t ret;
    mb_uint16_t req_nbytes;
    mb_uint16_t rsp_nbytes;
    mb_int16_t rsp_len_computed;

    if ((slave_id != mb->slave_id) && (slave_id != MODBUS_ADDR_BROADCAST)) {
        ret = -MB_EBADSLAVE;
        goto err_exit;
    }

    /* Exception code */
    if (rsp[0] > 0x80) {
        if (req[0] == (rsp[0] - 0x80)) {
            if (rsp[1] < MB_EX_MAX) {
                ret = -rsp[1];
            } else {
                ret = -MB_EBADEXC;
            }
        } else {
            ret = -MB_EBADEXC;
        }

        goto err_exit;
    }

    /* Check length */
    rsp_len_computed = compute_rsp_len_from_req(req);

    if (((rsp_len == rsp_len_computed) || (rsp_len_computed == -1))
        && (rsp[0] < 0x80)) {
        /* Check function code */
        if (rsp[0] != req[0]) {
            MODBUS_DBG("Received function not corresponding to the request (0x%X != 0x%X)\n",
                       rsp[0], req[0]);
            ret = -MB_EBADDATA;
            goto err_exit;
        }

        /* Check the number of values is corresponding to the request */
        switch (rsp[0]) {
            case MODBUS_FC_READ_COILS:
            case MODBUS_FC_READ_DISCRETE_INPUTS:
                /* Read functions, 8 values in a byte (nb
                 * of values in the request and byte count in
                 * the response. */
                req_nbytes = (req[3] << 8) + req[4];
                req_nbytes = (req_nbytes / 8) + ((req_nbytes % 8) ? 1 : 0);
                rsp_nbytes = rsp[1];
                break;

            case MODBUS_FC_WRITE_AND_READ_REGISTERS:
            case MODBUS_FC_READ_HOLDING_REGISTERS:
            case MODBUS_FC_READ_INPUT_REGISTERS:
                /* Read functions 1 value = 2 bytes */
                req_nbytes = (req[3] << 8) + req[4];
                rsp_nbytes = (rsp[1] / 2);
                break;

            case MODBUS_FC_WRITE_MULTIPLE_COILS:
            case MODBUS_FC_WRITE_MULTIPLE_REGISTERS:
                /* N Write functions */
                req_nbytes = (req[3] << 8) + req[4];
                rsp_nbytes = (rsp[3] << 8) | rsp[4];
                break;

            case MODBUS_FC_REPORT_SLAVE_ID:
                /* Report slave ID (bytes received) */
                req_nbytes = rsp_nbytes = rsp[1];
                break;

            default:
                /* 1 Write functions & others */
                req_nbytes = rsp_nbytes = 1;
                break;
        }

        if (req_nbytes == rsp_nbytes) {
            ret = MB_ENOERR;
        } else {
            MODBUS_DBG("Quantity not corresponding to the request (%d != %d)\n",
                       rsp_nbytes, req_nbytes);
            ret = -MB_EBADDATA;
        }
    } else {
        MODBUS_DBG("Message length not corresponding to the computed length (%d != %d)\n",
                   rsp_len, rsp_len_computed);
        ret = -MB_EBADDATA;
    }

    err_exit:
    return ret;
}

/**
  \brief  Reads IO status
*/
#if ((MB_FUNC_READ_COILS_ENABLED > 0) || (MB_FUNC_READ_DISCRETE_INPUTS_ENABLED > 0))
static mb_err_t _mb_read_io_status(modbus_t *mb, mb_uint8_t function,
    mb_uint16_t reg_addr, mb_uint16_t ncoils, mb_uint8_t *rsp)
{
    mb_err_t ret;
    mb_uint8_t rsp_slaveid;
    mb_uint8_t *rsp_frame;
    mb_uint16_t rsp_length;
    mb_uint8_t req_frame[MODBUS_MIN_REQ_LENGTH];
    mb_uint16_t i;
    mb_uint8_t bit;
    mb_uint16_t pos = 0;

    if ((mb == MB_NULL) || (rsp == MB_NULL))
    {
        ret = -MB_EILVAL;
        goto err_exit;
    }

    req_frame[0] = function;
    req_frame[1] = (mb_uint8_t)(reg_addr >> 8);
    req_frame[2] = (mb_uint8_t)(reg_addr & 0x00FF);
    req_frame[3] = (mb_uint8_t)(ncoils >> 8);
    req_frame[4] = (mb_uint8_t)(ncoils & 0x00FF);
    ret = modbus_send(mb, mb->slave_id, req_frame, 5);

    if (ret != MB_ENOERR)
    {
        goto err_exit;
    }

    ret = modbus_recv(mb, &rsp_slaveid, &rsp_frame, &rsp_length);

    if (ret != MB_ENOERR)
    {
        goto err_exit;
    }

    ret = _mb_check_confirm(mb, req_frame, rsp_slaveid, rsp_frame, rsp_length);

    if (ret != MB_ENOERR)
    {
        goto err_exit;
    }

    for (i = 0; i < rsp_frame[1]; i++)
    {
        /* Shift reg hi_byte to temp */
        for (bit = 0x01; (bit != 0x00) && (pos < ncoils);)
        {
            rsp[pos++] = (rsp_frame[2 + i] & bit) ? 1 : 0;
            bit = bit << 1;
        }
    }
    ret = MB_ENOERR;
err_exit:
    return ret;
}
#endif


/**
  \brief  Reads the data from a remove device and put that data into an array
*/
#if ((MB_FUNC_READ_HOLDING_ENABLED > 0) || (MB_FUNC_READ_INPUT_ENABLED > 0))

static mb_err_t _mb_read_regs(modbus_t* mb, mb_uint8_t function,
                              mb_uint16_t reg_addr, mb_uint16_t nregs, mb_uint16_t* rsp) {
    mb_err_t ret;
    mb_uint16_t cnt;
    mb_uint8_t rsp_slaveid;
    mb_uint8_t* rsp_frame;
    mb_uint16_t rsp_length;
    mb_uint8_t req_frame[MODBUS_MIN_REQ_LENGTH];

    if ((mb == MB_NULL) || (rsp == MB_NULL)) {
        ret = -MB_EILVAL;
        goto err_exit;
    }

    if (nregs > MODBUS_MAX_READ_REGISTERS) {
        MODBUS_DBG("ERROR Too many registers requested (%d > %d)\n",
                   nregs, MODBUS_MAX_READ_REGISTERS);
        ret = -MB_EMDATA;
        goto err_exit;
    }

    req_frame[0] = function;
    req_frame[1] = (mb_uint8_t) (reg_addr >> 8);
    req_frame[2] = (mb_uint8_t) (reg_addr & 0x00FF);
    req_frame[3] = (mb_uint8_t) (nregs >> 8);
    req_frame[4] = (mb_uint8_t) (nregs & 0x00FF);
    ret = modbus_send(mb, mb->slave_id, req_frame, 5);

    if (ret != MB_ENOERR) {
        goto err_exit;
    }

    ret = modbus_recv(mb, &rsp_slaveid, &rsp_frame, &rsp_length);

    if (ret != MB_ENOERR) {
        goto err_exit;
    }

    ret = _mb_check_confirm(mb, req_frame, rsp_slaveid, rsp_frame, rsp_length);

    if (ret != MB_ENOERR) {
        goto err_exit;
    }

    for (cnt = 0; cnt < nregs; cnt++) {
        /* Shift reg hi_byte to temp OR with lo_byte */
        rsp[cnt] = (rsp_frame[2 + (cnt << 1)] << 8) |
                   rsp_frame[3 + (cnt << 1)];
    }
    ret = MB_ENOERR;
    err_exit:
    return ret;
}

#endif  // #if ((MB_FUNC_READ_HOLDING_ENABLED > 0) || (MB_FUNC_READ_INPUT_ENABLED > 0))

/**
  \brief  Write a value to the specified register of the remote device.
          Used by write_bit and write_register
*/
#if ((MB_FUNC_WRITE_COIL_ENABLED > 0) || (MB_FUNC_WRITE_HOLDING_ENABLED > 0))

static mb_err_t _mb_write_single(modbus_t* mb,
                                 mb_uint8_t function, mb_uint16_t reg_addr, mb_uint16_t value) {
    mb_err_t ret;
    mb_uint8_t rsp_slaveid;
    mb_uint8_t* rsp_frame;
    mb_uint16_t rsp_length;
    mb_uint8_t req_frame[MODBUS_MIN_REQ_LENGTH];

    if (mb == MB_NULL) {
        ret = -MB_EILVAL;
        goto err_exit;
    }

    req_frame[0] = function;
    req_frame[1] = (mb_uint8_t) (reg_addr >> 8);
    req_frame[2] = (mb_uint8_t) (reg_addr & 0x00FF);
    req_frame[3] = (mb_uint8_t) (value >> 8);
    req_frame[4] = (mb_uint8_t) (value & 0x00FF);
    ret = modbus_send(mb, mb->slave_id, req_frame, 5);
    if (ret != MB_ENOERR) {
        goto err_exit;
    }

    ret = modbus_recv(mb, &rsp_slaveid, &rsp_frame, &rsp_length);
    if (ret != MB_ENOERR) {
        goto err_exit;
    }

    ret = _mb_check_confirm(mb, req_frame, rsp_slaveid, rsp_frame, rsp_length);
    err_exit:
    return ret;
}

#endif  // #if ((MB_FUNC_WRITE_COIL_ENABLED > 0) || (MB_FUNC_WRITE_HOLDING_ENABLED > 0))
#endif  // #if ((MODBUS_DEVICE_MODE == 1) || (MODBUS_DEVICE_MODE == 3))

/**
  \fn          modbus_t *modbus_create(mb_uint8_t conn_mode, void *port)
  \brief       Create a Modbus object
  \param[in]   conn_mode    Modbus connection mode, ascii/rtu/any
  \param[in]   port         A pointer to modbus port object
  \return      >0 : Success, 0 : Fail
*/
modbus_t* modbus_create(mb_uint8_t conn_mode, void* port) {
    mb_int32_t index;
    modbus_t* mb = MB_NULL;

    if (port == MB_NULL) {
        goto err_exit;
    }

    /* Find an empty modbus entry */
    for (index = 0; index < MODBUS_OBJECT_NUM; index++) {
        mb = (modbus_t*) &modbus_table[index];

        if (mb->magic != MODBUS_MAGIC) {
            break;
        }
    }

    if (index == MODBUS_OBJECT_NUM) {
        MODBUS_DBG("Modbus object creation failed and memory resources could not be allocated.\n");
        mb = MB_NULL;
        goto err_exit;
    }

    modbus_memset(mb, 0, sizeof(modbus_t));
    mb->magic = MODBUS_MAGIC;
    mb->mode = conn_mode;
    mb->rcv_tmo = 3000;
    mb->port = (modbus_port_t*) port;
    //MODBUS_DBG("Modbus create successful.\n");
    err_exit:
    return mb;
}

/**
  \fn          mb_err_t modbus_destroy(modbus_t *mb)
  \brief       Destroy a Modbus object
  \param[in]   mb    A pointer to Modbus objcet
  \return      0 : Success, <0 : Fail
*/
mb_err_t modbus_destroy(modbus_t* mb) {
    mb_err_t ret;

    if (mb == MB_NULL) {
        ret = -MB_EILVAL;
        goto err_exit;
    }

    /* Disconnect */
    if (mb->state == MODBUS_FLAG_ENABLED) {
        if ((mb->port == MB_NULL) || (mb->port->close_fn == MB_NULL)) {
            ret = -MB_EIO;
            goto err_exit;
        }
        ret = mb->port->close_fn(mb->port);
        if (ret != MB_ENOERR)
            goto err_exit;
    }

    modbus_memset(mb, 0, sizeof(modbus_t));
    //MODBUS_DBG("Modbus object destruction successful.\n");
    return MB_ENOERR;
    err_exit:
    MODBUS_DBG("Modbus object destruction failed. errcode = %d\n", ret);
    return ret;
}

/**
  \fn          mb_err_t modbus_connect(modbus_t *mb)
  \brief       Modbus conncet
  \param[in]   mb        A pointer to Modbus objcet
  \return      0 : Success, <0 : Fail
*/
mb_err_t modbus_connect(modbus_t* mb) {
    mb_err_t ret = MB_ENOERR;

    if (mb == MB_NULL) {
        ret = -MB_EILVAL;
        goto err_exit;
    }

    if (mb->state == MODBUS_FLAG_DISABLED) {
        if ((mb->port == MB_NULL) && (mb->port->open_fn == MB_NULL)) {
            ret = -MB_EIO;
            goto err_exit;
        }

        ret = mb->port->open_fn(mb->port);
        if (ret != MB_ENOERR) {
            goto err_exit;
        }

        mb->state = MODBUS_FLAG_ENABLED;
        //MODBUS_DBG("Modbus connect succeeded.\n");
        return MB_ENOERR;
    } else {
        ret = -MB_EILLSTATE;
    }

    err_exit:
    MODBUS_DBG("Modbus connect failed.\n");
    return ret;
}

/**
  \fn          mb_err_t modbus_disconn(modbus_t *mb)
  \brief       Modbus disconnect
  \param[in]   mb    A pointer to Modbus objcet
  \return      0 : Success, <0 : Fail
*/
mb_err_t modbus_disconn(modbus_t* mb) {
    mb_err_t ret = MB_ENOERR;

    if (mb == MB_NULL) {
        ret = -MB_EILVAL;
        goto err_exit;
    }

    if (mb->state == MODBUS_FLAG_ENABLED) {
        if ((mb->port == MB_NULL) || (mb->port->close_fn == MB_NULL)) {
            ret = -MB_EIO;
            goto err_exit;
        }

        ret = mb->port->close_fn(mb->port);
        if (ret != MB_ENOERR) {
            MODBUS_DBG("Modbus disconnect failed.\n");
            goto err_exit;
        }
        //MODBUS_DBG("Modbus disconnect successful.\n");
        mb->state = MODBUS_FLAG_DISABLED;
    } else {
        MODBUS_DBG("Modbus is disconnected.\n");
        ret = -MB_EILLSTATE;
    }
    err_exit:
    return ret;
}

/**
  \fn          mb_err_t modbus_control(modbus_t *mb, uint32_t cmd, void *args)
  \brief       Modbus control
  \param[in]   mb    A pointer to Modbus objcet
  \param[in]   cmd   Configurate command
  \param[in]   args  Configurate paramer
  \return      0 : Success, <0 : Fail
*/
mb_err_t modbus_control(modbus_t* mb, mb_uint32_t cmd, void* args) {
    mb_err_t ret;

    if (mb == MB_NULL) {
        ret = -MB_EILVAL;
        goto err_exit;
    }

    if ((mb->port == MB_NULL) || (mb->port->ctrl_fn == MB_NULL)) {
        ret = -MB_EIO;
        goto err_exit;
    }

    ret = mb->port->ctrl_fn(mb->port, cmd, args);
    err_exit:
    return ret;
}

/**
  \fn          mb_err_t modbus_send(modbus_t *mb, mb_uint8_t slave_id,
                   const mb_uint8_t *frame, mb_uint16_t length)
  \brief       Send a message
  \param[in]   mb        A pointer to Modbus objcet
  \param[in]   slave_id  Slave identifier
  \param[in]   frame     Modbus request frame to send
  \param[in]   length    Modbus requests frame length
  \return      0 : Success, <0 : Fail
*/
mb_err_t modbus_send(modbus_t* mb, mb_uint8_t slave_id,
                     const mb_uint8_t* frame, mb_uint16_t length) {
    mb_err_t ret;

    if (mb == MB_NULL) {
        ret = -MB_EILVAL;
        goto err_exit;
    }

    if ((mb->port == MB_NULL) || (mb->port->send_fn == MB_NULL)) {
        ret = -MB_EIO;
        goto err_exit;
    }

    switch (mb->mode) {
#if (MODBUS_ASCII_ENABLED > 0)
        case MODBUS_MODE_ASCII:
            ret = _mb_ascii_send(mb, slave_id, frame, length);
            break;
#endif
#if (MODBUS_RTU_ENABLED > 0)
        case MODBUS_MODE_RTU:
            ret = _mb_rtu_send(mb, slave_id, frame, length);
            break;
#endif
#if (MODBUS_TCPUDP_ENABLED > 0)
            case MODBUS_MODE_NONE:
                ret = _mb_net_send(mb, slave_id, frame, length);
                break;
#endif
        default:
            ret = -MB_EIO;
            break;
    }

    err_exit:
    return ret;
}

/**
  \fn          mb_err_t modbus_recv(modbus_t *mb, mb_uint8_t *slave_id,
                   mb_uint8_t **pframe, mb_uint16_t *length)
  \brief       Recive a message
  \param[in]   mb        Modbus objcet
  \param[in]   slave_id  A pointer to slave identifier
  \param[in]   pframe    Modbus reply frame to receive
  \param[in]   length    Modbus reply frame length
  \return      0 :  Success, <0 : Fail
*/
mb_err_t modbus_recv(modbus_t* mb, mb_uint8_t* slave_id,
                     mb_uint8_t** pframe, mb_uint16_t* length) {
    mb_err_t ret;

    if ((mb == MB_NULL) || (pframe == MB_NULL) || (length == MB_NULL)) {
        ret = -MB_EILVAL;
        goto err_exit;
    }

    if ((mb->port == MB_NULL) || (mb->port->recv_fn == MB_NULL)) {
        ret = -MB_EIO;
        goto err_exit;
    }

    switch (mb->mode) {
#if (MODBUS_ASCII_ENABLED > 0)
        case MODBUS_MODE_ASCII:
            ret = _mb_ascii_recv(mb, slave_id, pframe, length);
            break;
#endif
#if (MODBUS_RTU_ENABLED > 0)
        case MODBUS_MODE_RTU:
            ret = _mb_rtu_recv(mb, slave_id, pframe, length);
            break;
#endif
#if (MODBUS_TCPUDP_ENABLED > 0)
            case MODBUS_MODE_NONE:
                ret = _mb_net_recv(mb, slave_id, pframe, length);
                break;
#endif
        default:
            ret = -MB_EIO;
            break;
    }

    err_exit:
    return ret;
}

/**
  \fn          mb_err_t modbus_set_slave(modbus_t *mb, mb_uint8_t slave_id)
  \brief       Define the slave identifier
  \param[in]   mb        Modbus objcet
  \param[in]   slave_id  Slave identifier
  \return      0 :  Success, <0 : Fail
*/
mb_err_t modbus_set_slave(modbus_t* mb, mb_uint8_t slave_id) {
    if ((mb == MB_NULL) || (slave_id > MODBUS_ADDR_MAX)) {
        return -MB_EILVAL;
    }

    mb->slave_id = slave_id;
    return MB_ENOERR;
}

/**
  \fn          mb_err_t modbus_set_recvtmo(modbus_t *mb, mb_uint32_t timeout)
  \brief       Set modbus receive timeout
  \param[in]   mb        Modbus objcet
  \param[in]   timeout   Receive timeout (in ms)
  \return      0 :  Success, <0 : Fail
*/
mb_err_t modbus_set_recvtmo(modbus_t* mb, mb_uint32_t time_ms) {
    if (mb == MB_NULL) {
        return -MB_EILVAL;
    }

    mb->rcv_tmo = time_ms;
    return MB_ENOERR;
}

#if ((MODBUS_DEVICE_MODE == 1) || (MODBUS_DEVICE_MODE == 3))
/**
  \fn          mb_err_t modbus_read_bits(modbus_t *mb,
                   mb_uint16_t reg_addr, mb_uint16_t ncoils, mb_uint8_t *rsp)
  \brief       Reads the boolean status of bits and sets the array elements
                   in the destination to TRUE or FALSE (single bits).
  \param[in]   mb        A pointer to Modbus objcet
  \param[in]   reg_addr  Register address
  \param[in]   ncoils    Number of coils
  \param[in]   rsp       The buffer pointer that holds the response data
  \return      0 : Success, <0 : Fail
*/
#if (MB_FUNC_READ_COILS_ENABLED > 0)
mb_err_t modbus_read_bits(modbus_t *mb,
    mb_uint16_t reg_addr, mb_uint16_t ncoils, mb_uint8_t *rsp)
{
    mb_err_t ret;

    if (mb == MB_NULL)
    {
        ret = -MB_EILVAL;
        goto err_exit;
    }

    if (ncoils > MODBUS_MAX_READ_BITS)
    {
        MODBUS_DBG("ERROR Too many bits requested (%d > %d)\n",
                    ncoils, MODBUS_MAX_READ_BITS);
        ret = -MB_EMDATA;
        goto err_exit;
    }

    ret = _mb_read_io_status(mb, MODBUS_FC_READ_COILS, reg_addr, ncoils, rsp);
err_exit:
    return ret;
}
#endif

/**
  \fn          mb_err_t modbus_read_input_bits(modbus_t *mb,
                   mb_uint16_t reg_addr, mb_uint16_t ncoils, mb_uint8_t *rsp)
  \brief       Same as modbus_read_bits but reads the remote device input table
  \param[in]   mb        A pointer to Modbus objcet
  \param[in]   reg_addr  Register address
  \param[in]   ncoils    Number of coils
  \param[in]   rsp       The buffer pointer that holds the response data
  \return      0 : Success, <0 : Fail
*/
#if (MB_FUNC_READ_DISCRETE_INPUTS_ENABLED > 0)
mb_err_t modbus_read_input_bits(modbus_t *mb,
    mb_uint16_t reg_addr, mb_uint16_t ncoils, mb_uint8_t *rsp)
{
    mb_err_t ret;

    if (mb == MB_NULL)
    {
        ret = -MB_EILVAL;
        goto err_exit;
    }

    if (ncoils > MODBUS_MAX_READ_BITS)
    {
        MODBUS_DBG("ERROR Too many bits requested (%d > %d)\n",
                    ncoils, MODBUS_MAX_READ_BITS);
        ret = -MB_EMDATA;
        goto err_exit;
    }

    ret = _mb_read_io_status(mb, MODBUS_FC_READ_DISCRETE_INPUTS, reg_addr, ncoils, rsp);

err_exit:
    return ret;
}
#endif

/**
  \fn          mb_err_t modbus_read_holding_regs(modbus_t *mb, mb_uint8_t slaveid,
                   mb_uint16_t reg_addr, mb_uint16_t nregs, mb_uint16_t *resp)
  \brief       Reads the holding registers of remote device and put the data into an
               array
  \param[in]   mb        A pointer to Modbus objcet
  \param[in]   reg_addr  Register address
  \param[in]   nregs     Register number
  \param[in]   rsp       The buffer pointer that holds the response data
  \return      0 : Success, <0 : Fail
*/
#if (MB_FUNC_READ_HOLDING_ENABLED > 0)

mb_err_t modbus_read_holding_regs(modbus_t* mb,
                                  mb_uint16_t reg_addr, mb_uint16_t nregs, mb_uint16_t* rsp) {
    return _mb_read_regs(mb, MODBUS_FC_READ_HOLDING_REGISTERS,
                         reg_addr, nregs, rsp);
}

#endif

/**
  \fn          mb_err_t modbus_read_holding_regs(modbus_t *mb,
                   mb_uint16_t reg_addr, mb_uint16_t nregs, mb_uint16_t *resp)
  \brief       Reads the input registers of remote device and put the data into an array
  \param[in]   mb        A pointer to Modbus objcet
  \param[in]   reg_addr  Register address
  \param[in]   nregs     Register number
  \param[in]   rsp       The buffer pointer that holds the response data
  \return      0 : Success, <0 : Fail
*/
#if (MB_FUNC_READ_INPUT_ENABLED > 0)

mb_err_t modbus_read_input_regs(modbus_t* mb,
                                mb_uint16_t reg_addr, mb_uint16_t nregs, mb_uint16_t* rsp) {
    return _mb_read_regs(mb, MODBUS_FC_READ_INPUT_REGISTERS,
                         reg_addr, nregs, rsp);
}

#endif

/**
  \fn          mb_err_t modbus_write_coil(modbus_t *mb,
                   mb_uint16_t reg_addr, mb_uint8_t status)
  \brief       Turns ON or OFF a single bit of the remote device
  \param[in]   mb        A pointer to Modbus objcet
  \param[in]   reg_addr  Register address
  \param[in]   status    0 : OFF of !0 : ON
  \return      0 : Success, <0 : Fail
*/
#if (MB_FUNC_WRITE_COIL_ENABLED > 0)
mb_err_t modbus_write_coil(modbus_t *mb,
    mb_uint16_t reg_addr, mb_uint8_t status)
{
    return _mb_write_single(mb, MODBUS_FC_WRITE_SINGLE_COIL,
            reg_addr, status ? 0xFF00 : 0);
}
#endif

/**
  \fn          mb_err_t modbus_write_register(modbus_t *mb,
                   mb_uint16_t reg_addr, mb_uint16_t value)
  \brief       Writes a value in one register of the remote device
  \param[in]   mb        A pointer to Modbus objcet
  \param[in]   reg_addr  Register address
  \param[in]   value     The value to be written
  \return      0 : Success, <0 : Fail
*/
#if (MB_FUNC_WRITE_HOLDING_ENABLED > 0)

mb_err_t modbus_write_register(modbus_t* mb,
                               mb_uint16_t reg_addr, mb_uint16_t value) {
    return _mb_write_single(mb, MODBUS_FC_WRITE_SINGLE_REGISTER,
                            reg_addr, value);
}

#endif

/**
  \fn          mb_err_t modbus_write_coils(modbus_t *mb,
                   mb_uint16_t reg_addr, mb_uint16_t nregs, const mb_uint8_t *src)
  \brief       Write the bits of the array in the remote device
  \param[in]   mb        A pointer to Modbus objcet
  \param[in]   reg_addr  Register address
  \param[in]   nregs     Number of registers
  \param[in]   req       A pointer to data buffer
  \return      0 : Success, <0 : Fail
*/
#if (MB_FUNC_WRITE_MULTIPLE_COILS_ENABLED > 0)
mb_err_t modbus_write_coils(modbus_t *mb,
    mb_uint16_t reg_addr, mb_uint16_t nregs, const mb_uint8_t *req)
{
    mb_err_t ret;
    mb_uint16_t i, byte_count;
    mb_uint8_t rsp_slaveid;
    mb_uint8_t *rsp_frame;
    mb_uint16_t rsp_length;
    mb_uint8_t req_frame[MODBUS_ADU_SIZE_MAX];
    mb_uint16_t req_length;
    mb_uint16_t pos = 0;
    mb_uint16_t bit;
    mb_uint16_t bit_check = 0;

    if (mb == MB_NULL)
    {
        ret = -MB_EILVAL;
        goto err_exit;
    }

    if (nregs > MODBUS_MAX_WRITE_BITS)
    {
        MODBUS_DBG("ERROR Writing too many bits (%d > %d)\n",
                    nregs, MODBUS_MAX_WRITE_BITS);
        ret = -MB_EMDATA;
        goto err_exit;
    }

    req_frame[0] = MODBUS_FC_WRITE_MULTIPLE_COILS;
    req_frame[1] = (mb_uint8_t)(reg_addr >> 8);
    req_frame[2] = (mb_uint8_t)(reg_addr & 0x00FF);
    req_frame[3] = (mb_uint8_t)(nregs >> 8);
    req_frame[4] = (mb_uint8_t)(nregs & 0x00FF);

    req_length = 5;
    byte_count = (nregs / 8) + ((nregs % 8) ? 1 : 0);
    req_frame[req_length++] = byte_count;

    for (i = 0; i < byte_count; i++)
    {
        bit = 0x01;
        req_frame[req_length] = 0;

        while ((bit & 0xFF) && (bit_check++ < nregs))
        {
            if (req[pos++] != 0)
                req_frame[req_length] |= bit;
            else
                req_frame[req_length] &= ~ bit;

            bit = bit << 1;
        }
        req_length++;
    }

    ret = modbus_send(mb, mb->slave_id, req_frame, req_length);
    if (ret != MB_ENOERR)
    {
        goto err_exit;
    }

    ret = modbus_recv(mb, &rsp_slaveid, &rsp_frame, &rsp_length);
    if (ret != MB_ENOERR)
    {
        goto err_exit;
    }

    ret = _mb_check_confirm(mb, req_frame, rsp_slaveid, rsp_frame, rsp_length);

err_exit:
    return ret;
}
#endif

/**
  \fn          mb_err_t modbus_write_registers(modbus_t *mb,
                   mb_uint16_t reg_addr, mb_uint16_t nregs, const mb_uint16_t *src)
  \brief       Write the values from the array to the registers of the remote device
  \param[in]   mb        A pointer to Modbus objcet
  \param[in]   reg_addr  Register address
  \param[in]   nregs     Register number
  \param[in]   req       A pointer to data buffer
  \return      0 : Success, <0 : Fail
*/
#if (MB_FUNC_WRITE_MULTIPLE_HOLDING_ENABLED > 0)
mb_err_t modbus_write_registers(modbus_t *mb,
    mb_uint16_t reg_addr, mb_uint16_t nregs, const mb_uint16_t *req)
{
    mb_err_t ret;
    mb_uint16_t i, byte_count;
    mb_uint8_t rsp_slaveid;
    mb_uint8_t *rsp_frame;
    mb_uint16_t rsp_length;
    mb_uint8_t req_frame[MODBUS_ADU_SIZE_MAX];
    mb_uint16_t req_length;

    if (mb == MB_NULL)
    {
        ret = -MB_EILVAL;
        goto err_exit;
    }

    if (nregs > MODBUS_MAX_WRITE_REGISTERS)
    {
        MODBUS_DBG("ERROR Trying to write to too many registers (%d > %d)\n",
                    nregs, MODBUS_MAX_WRITE_REGISTERS);
        ret = -MB_EMDATA;
        goto err_exit;
    }

    req_frame[0] = MODBUS_FC_WRITE_MULTIPLE_REGISTERS;
    req_frame[1] = (mb_uint8_t)(reg_addr >> 8);
    req_frame[2] = (mb_uint8_t)(reg_addr & 0x00FF);
    req_frame[3] = (mb_uint8_t)(nregs >> 8);
    req_frame[4] = (mb_uint8_t)(nregs & 0x00FF);

    req_length = 5;
    byte_count = nregs * 2;
    req_frame[req_length++] = byte_count;

    for (i = 0; i < nregs; i++)
    {
        req_frame[req_length++] = req[i] >> 8;
        req_frame[req_length++] = req[i] & 0x00FF;
    }

    ret = modbus_send(mb, mb->slave_id, req_frame, req_length);
    if (ret != MB_ENOERR)
    {
        goto err_exit;
    }

    ret = modbus_recv(mb, &rsp_slaveid, &rsp_frame, &rsp_length);
    if (ret != MB_ENOERR)
    {
        goto err_exit;
    }

    ret = _mb_check_confirm(mb, req_frame, rsp_slaveid, rsp_frame, rsp_length);

err_exit:
    return ret;
}
#endif

/**
  \fn          mb_err_t modbus_mask_write_register(modbus_t *mb,
                    mb_uint16_t reg_addr, mb_uint16_t and_mask, mb_uint16_t or_mask)
  \brief       Mask write the values to the registers of the remote device
  \param[in]   mb        A pointer to Modbus objcet
  \param[in]   reg_addr  Register address
  \param[in]   and_mask  And mask
  \param[in]   or_mask   OR mask
  \return      0 : Success, <0 : Fail
*/
#if (MB_FUNC_MASK_WRITE_ENABLED > 0)
mb_err_t modbus_mask_write_register(modbus_t *mb,
    mb_uint16_t reg_addr, mb_uint16_t and_mask, mb_uint16_t or_mask)
{
    mb_err_t ret;
    mb_uint8_t rsp_slaveid;
    mb_uint8_t *rsp_frame;
    mb_uint16_t rsp_length;
    mb_uint8_t req_frame[MODBUS_MIN_REQ_LENGTH];

    if (mb == MB_NULL)
    {
        ret = -MB_EILVAL;
        goto err_exit;
    }

    req_frame[0] = MODBUS_FC_MASK_WRITE_REGISTER;
    req_frame[1] = (mb_uint8_t)(reg_addr >> 8);
    req_frame[2] = (mb_uint8_t)(reg_addr & 0x00FF);
    req_frame[3] = (mb_uint8_t)(and_mask >> 8);
    req_frame[4] = (mb_uint8_t)(and_mask & 0x00FF);
    req_frame[5] = (mb_uint8_t)(or_mask >> 8);
    req_frame[6] = (mb_uint8_t)(or_mask & 0x00FF);

    ret = modbus_send(mb, mb->slave_id, req_frame, 7);
    if (ret != MB_ENOERR)
    {
        goto err_exit;
    }

    ret = modbus_recv(mb, &rsp_slaveid, &rsp_frame, &rsp_length);
    if (ret != MB_ENOERR)
    {
        goto err_exit;
    }

    ret = _mb_check_confirm(mb, req_frame, rsp_slaveid, rsp_frame, rsp_length);

err_exit:
    return ret;
}
#endif

/**
  \fn          mb_err_t modbus_mask_write_register(modbus_t *mb,
                    mb_uint16_t reg_addr, mb_uint16_t and_mask, mb_uint16_t or_mask)
  \brief       Write multiple registers from src array to remote device
                and read multiple registers from remote device to dest array
  \param[in]   mb         A pointer to Modbus objcet
  \param[in]   read_addr  Read register address
  \param[in]   read_nb    Number of read register
  \param[in]   dest       A pointer to read data buffer
  \param[in]   write_addr Write register address
  \param[in]   write_nb   Number of write register
  \param[in]   src        A pointer to write data buffer
  \return      0 : Success, <0 : Fail
*/
#if (MB_FUNC_READWRITE_HOLDING_ENABLED > 0)
mb_err_t modbus_write_and_read_registers(modbus_t *mb,
    mb_uint16_t read_addr, mb_uint16_t read_nb, mb_uint16_t *rsp,
    mb_uint16_t write_addr, mb_uint16_t write_nb, const mb_uint8_t *req)

{
    mb_err_t ret;
    mb_uint16_t i;
    mb_uint8_t byte_count;
    mb_uint8_t rsp_slaveid;
    mb_uint8_t *rsp_frame;
    mb_uint16_t rsp_length;
    mb_uint8_t req_length;
    mb_uint8_t req_frame[MODBUS_ADU_SIZE_MAX];

    if (mb == MB_NULL)
    {
        ret = -MB_EILVAL;
        goto err_exit;
    }

    if (write_nb > MODBUS_MAX_WR_WRITE_REGISTERS)
    {
        MODBUS_DBG("ERROR Too many registers to write (%d > %d)\n",
                    write_nb, MODBUS_MAX_WRITE_REGISTERS);
        ret = -MB_EMDATA;
        goto err_exit;
    }

    if (read_nb > MODBUS_MAX_WR_READ_REGISTERS)
    {
        MODBUS_DBG("ERROR Too many registers requested (%d > %d)\n",
                    read_nb, MODBUS_MAX_WR_READ_REGISTERS);
        ret = -MB_EMDATA;
        goto err_exit;
    }

    req_frame[0] = MODBUS_FC_WRITE_AND_READ_REGISTERS;
    req_frame[1] = (mb_uint8_t)(read_addr >> 8);
    req_frame[2] = (mb_uint8_t)(read_addr & 0x00FF);
    req_frame[3] = (mb_uint8_t)(read_nb >> 8);
    req_frame[4] = (mb_uint8_t)(read_nb & 0x00FF);
    req_frame[5] = (mb_uint8_t)(write_addr >> 8);
    req_frame[6] = (mb_uint8_t)(write_addr & 0x00FF);
    req_frame[7] = (mb_uint8_t)(write_nb >> 8);
    req_frame[8] = (mb_uint8_t)(write_nb & 0x00FF);
    req_frame[9] = write_nb * 2;
    req_length = 10;
    for (i = 0; i < write_nb; i++)
    {
        req_frame[req_length++] = (mb_uint8_t)(req[i] >> 8);
        req_frame[req_length++] = (mb_uint8_t)(req[i] & 0x00FF);
    }

    ret = modbus_send(mb, mb->slave_id, req_frame, req_length);
    if (ret != MB_ENOERR)
    {
        goto err_exit;
    }

    ret = modbus_recv(mb, &rsp_slaveid, &rsp_frame, &rsp_length);
    if (ret != MB_ENOERR)
    {
        goto err_exit;
    }

    ret = _mb_check_confirm(mb, req_frame, rsp_slaveid, rsp_frame, rsp_length);
    if (ret != MB_ENOERR)
    {
        goto err_exit;
    }

    byte_count = rsp_frame[1] * 2;
    for (i = 0; i < byte_count; i++)
    {
        /* shift reg hi_byte to temp OR with lo_byte */
        rsp[i] = (req_frame[2 + (i * 2)] << 8) | req_frame[3 + (i * 2)];
    }
    ret = MB_ENOERR;

err_exit:
    return ret;
}
#endif

/**
  \fn          mb_err_t modbus_report_slave_id(modbus_t *mb,
                   mb_uint16_t max_rsp, mb_uint8_t *rsp)
  \brief       Send a request to get the slave ID of the device
               (only available in serial communication)
  \param[in]   mb         A pointer to Modbus objcet
  \param[in]   max_rsp    Maximum response buffer length
  \param[in]   rsp        A pointer to response buffer
  \return      0 : Success, <0 : Fail
*/
#if (MB_FUNC_REP_SLAVEID_ENABLED > 0)
mb_err_t modbus_report_slave_id(modbus_t *mb,
    mb_uint16_t max_rsp, mb_uint8_t *rsp)
{
    mb_err_t ret;
    mb_uint16_t i;
    mb_uint8_t rsp_slaveid;
    mb_uint8_t *rsp_frame;
    mb_uint16_t rsp_length;
    mb_uint8_t req_frame[MODBUS_MIN_REQ_LENGTH];

    if (mb == MB_NULL)
    {
        ret = -MB_EILVAL;
        goto err_exit;
    }

    req_frame[0] = MODBUS_FC_REPORT_SLAVE_ID;
    ret = modbus_send(mb, mb->slave_id, req_frame, 1);
    if (ret != MB_ENOERR)
    {
        goto err_exit;
    }

    ret = modbus_recv(mb, &rsp_slaveid, &rsp_frame, &rsp_length);
    if (ret != MB_ENOERR)
    {
        goto err_exit;
    }

    ret = _mb_check_confirm(mb, req_frame, rsp_slaveid, rsp_frame, rsp_length);
    if (ret != MB_ENOERR)
    {
        goto err_exit;
    }

    /* Byte count, slave id, run indicator status and
       additional data. Truncate copy to max_dest. */
    for (i = 0; i < rsp_frame[1] && i < max_rsp; i++)
    {
        rsp[i] = rsp_frame[2 + i];
    }

    ret = MB_ENOERR;
err_exit:
    return ret;
}
#endif
#endif // #if ((MODBUS_DEVICE_MODE == 1) || (MODBUS_DEVICE_MODE == 3))

/**
  \fn          mb_err_t modbus_set_mapping(modbus_t *mb, modbus_mapping_t *map)
  \brief       Define the modbus registers mapping
  \param[in]   mb   Modbus objcet
  \param[in]   map  A pointer to modbus registers mapping
  \return      0 :  Success, <0 : Fail
*/
mb_err_t modbus_set_mapping(modbus_t* mb, modbus_mapping_t* map) {
    if (mb == MB_NULL) {
        return -MB_EILVAL;
    }

    mb->map = map;
    return MB_ENOERR;
}

/**
  \fn          mb_err_t modbus_replay(modbus_t *mb, mb_uint8_t slave_id,
                   modbus_mapping_t *map)
  \brief       Recive a message and process, used for slave only.
  \param[in]   mb        A pointer to Modbus objcet
  \param[in]   slave_id  slave identifier
  \param[in]   map       Modbus register mapping
  \return      0 : Success, <0 : Fail
*/
mb_err_t modbus_replay(modbus_t* mb, mb_uint8_t slave_id,
                       mb_uint8_t* frame, mb_uint16_t length) {
    mb_uint8_t function;
    mb_err_t ret = MB_ENOERR;
    mb_exception_t ex = MB_EX_NONE;

    if ((mb == MB_NULL) || (mb->map == MB_NULL)) {
        return -MB_EILVAL;
    }

    if ((slave_id != mb->slave_id) &&
        (slave_id != MODBUS_ADDR_BROADCAST)) {
        return -MB_EBADDATA;
    }

    /* Get function code */
    function = frame[0];

    /* Data are flushed on illegal number of values errors. */
    switch (function) {
#if ((MODBUS_DEVICE_MODE == 2) || (MODBUS_DEVICE_MODE == 3))
#if (MB_FUNC_READ_COILS_ENABLED > 0)

        case MODBUS_FC_READ_COILS:
            ex = mbfunc_read_coils(mb, frame, &length);
            break;
#endif
#if (MB_FUNC_READ_DISCRETE_INPUTS_ENABLED > 0)

        case MODBUS_FC_READ_DISCRETE_INPUTS:
            ex = mbfunc_read_discrete_inputs(mb, frame, &length);
            break;
#endif
#if (MB_FUNC_READ_HOLDING_ENABLED > 0)

        case MODBUS_FC_READ_HOLDING_REGISTERS:
            ex = mbfunc_read_holding_registers(mb, frame, &length);
            break;
#endif
#if (MB_FUNC_READ_INPUT_ENABLED > 0)

        case MODBUS_FC_READ_INPUT_REGISTERS:
            ex = mbfunc_read_input_registers(mb, frame, &length);
            break;
#endif
#if (MB_FUNC_WRITE_COIL_ENABLED > 0)

        case MODBUS_FC_WRITE_SINGLE_COIL:
            ex = mbfunc_write_single_coil(mb, frame, &length);
            break;
#endif
#if (MB_FUNC_WRITE_HOLDING_ENABLED > 0)

        case MODBUS_FC_WRITE_SINGLE_REGISTER:
            ex = mbfunc_write_single_register(mb, frame, &length);
            break;
#endif

        case MODBUS_FC_READ_EXCEPTION_STATUS:
            /* Not implemented */
            return -MB_EILFUN;
#if (MB_FUNC_WRITE_MULTIPLE_COILS_ENABLED > 0)

        case MODBUS_FC_WRITE_MULTIPLE_COILS:
            ex = mbfunc_write_multiple_coils(mb, frame, &length);
            break;
#endif
#if (MB_FUNC_WRITE_MULTIPLE_HOLDING_ENABLED > 0)

        case MODBUS_FC_WRITE_MULTIPLE_REGISTERS:
            ex = mbfunc_write_multiple_registers(mb, frame, &length);
            break;
#endif
#if (MB_FUNC_REP_SLAVEID_ENABLED > 0)

        case MODBUS_FC_REPORT_SLAVE_ID:
            ex = mbfunc_report_slave_id(mb, frame, &length);
            break;
#endif
#if (MB_FUNC_MASK_WRITE_ENABLED > 0)

        case MODBUS_FC_MASK_WRITE_REGISTER:
            ex = mbfunc_mask_write_register(mb, frame, &length);
            break;
#endif
#if (MB_FUNC_READWRITE_HOLDING_ENABLED > 0)

        case MODBUS_FC_WRITE_AND_READ_REGISTERS:
            ex = mbfunc_write_and_read_registers(mb, frame, &length);
            break;
#endif
#endif  // #if ((MODBUS_DEVICE_MODE == 2) || (MODBUS_DEVICE_MODE == 3))  

        default:
            ex = MB_EX_ILLEGAL_FUNCTION;
            MODBUS_DBG("Unknown Modbus function code: 0x%0X\n", function);
            break;
    }

    if (ex != MB_EX_NONE) {
        length = 0;
        /* Build an exception response */
        frame[length++] = (mb_uint8_t) function | MODBUS_FC_ERROR;
        frame[length++] = ex;
    }

    /* Suppress any responses when the request was a broadcast */
    if (slave_id != MODBUS_ADDR_BROADCAST) {
        ret = modbus_send(mb, slave_id, frame, length);
    }

    return ret;
}

const char* modbus_strerr(mb_err_t errcode) {
    switch (-errcode) {
        case MB_EILFUN:
            return "Illegal function";
        case MB_EILADD:
            return "Illegal data address";
        case MB_EILVAL:
            return "Illegal data value";
        case MB_ESFAIL:
            return "Slave device or server failure";
        case MB_EACK:
            return "Acknowledge";
        case MB_ESBUSY:
            return "Slave device or server is busy";
        case MB_ENACK:
            return "Negative acknowledge";
        case MB_EMEMPAR:
            return "Memory parity error";
        case MB_ENOTDEF:
            return "Not defined";
        case MB_EGPATH:
            return "Gateway path unavailable";
        case MB_EGTAR:
            return "Target device failed to respond";
        case MB_EBADCRC:
            return "Invalid CRC";
        case MB_EBADDATA:
            return "Invalid data";
        case MB_EBADEXC:
            return "Invalid exception code";
        case MB_EMDATA:
            return "Too many data";
        case MB_EBADSLAVE:
            return "Response not from requested slave";
        case MB_ENORES:
            return "Insufficient resources.";
        case MB_EIO:
            return "I/O error.";
        case MB_ETIMEDOUT:
            return "Timeout error occurred.";
        case MB_EILLSTATE:
            return "Protocol stack in illegal state.";
    }
    return "";
}
