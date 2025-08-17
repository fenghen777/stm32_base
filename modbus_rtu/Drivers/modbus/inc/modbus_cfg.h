//<<< Use Configuration Wizard in Context Menu >>>
#ifndef __MB_CONFIG_H_
#define __MB_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

/*! \defgroup modbus_cfg Modbus Configuration
 *
 * Most modules in the protocol stack are completly optional and can be
 * excluded. This is specially important if target resources are very small
 * and program memory space should be saved.<br>
 *
 * All of these settings are available in the file <code>modbus_cfg.h</code>
 */
/*! \addtogroup modbus_cfg
 *  @{
 */

// <h>Modbus Global Configuration
// <i>Define modbus parameters

// <o> Modbus device mode <1=>Master <2=>Slave <3=>M&S
#define MODBUS_DEVICE_MODE                  1

//   <o>Number of modbus object <1-6>
//   <i>Define maximum number of modbus object that can be created at the same time.
//   <i>Default: 4
#define MODBUS_OBJECT_NUM                   2

//   <q>Modbus RTU mode
//   <i>Enable modbus RTU mode. 
#define MODBUS_RTU_ENABLED                  1

//   <q>Modbus ASCII mode
//   <i>Enable modbus ASCII mode. 
#define MODBUS_ASCII_ENABLED                0

//   <q>Modbus TCP/UDP mode
//   <i>Enable modbus TCP/UDP mode. 
#define MODBUS_TCPUDP_ENABLED               0

//   <q>Modbus uses the debugger
//   <i>Enable debug message.
#define MODBUS_USE_DEBUG                    1

#if (MODBUS_USE_DEBUG != 0)
#define MOBUS_DEBUG_PRINT                  printf
#endif
// </h>

// <h> Modbus Function Configuration
// <i> The protocol stack should support Modbus function code

//   <q>Read Coils
//   <i>If the Read Coils function should be enabled.
#define MB_FUNC_READ_COILS_ENABLED                  0

//   <q>Read Discrete Inputs
//   <i>If the Read Discrete Inputs function should be enabled.  
#define MB_FUNC_READ_DISCRETE_INPUTS_ENABLED        0

//   <q>Read Holding Registers
//   <i>If the Read Holding Registers function should be enabled.
#define MB_FUNC_READ_HOLDING_ENABLED                1

//   <q>Read Input Registers
//   <i>If the Read Input Registers function should be enabled. 
#define MB_FUNC_READ_INPUT_ENABLED                  1

//   <q>Write Single Coils
//   <i>If the Write Single Coils function should be enabled. 
#define MB_FUNC_WRITE_COIL_ENABLED                  0

//   <q>Write Single Register
//   <i>If the Write Single Register function should be enabled. 
#define MB_FUNC_WRITE_HOLDING_ENABLED               1

//   <q>Write Multiple Coils
//   <i>If the Write Multiple Coils function should be enabled.
#define MB_FUNC_WRITE_MULTIPLE_COILS_ENABLED        0

//   <q>Write Multiple registers
//   <i>If the Write Multiple registers function should be enabled.
#define MB_FUNC_WRITE_MULTIPLE_HOLDING_ENABLED      0

//   <q>Report Slave ID
//   <i>If the Report Slave ID function should be enabled. 
#define MB_FUNC_REP_SLAVEID_ENABLED                 0

//   <q>Mask Write Registers
//   <i>If the Mask Write Registers function should be enabled. 
#define MB_FUNC_MASK_WRITE_ENABLED                  0

//   <q>Read/Write multiple Registers
//   <i>If the Read/Write Multiple Registers function should be enabled. 
#define MB_FUNC_READWRITE_HOLDING_ENABLED           0

// </h>

// <h>Connection Configuration
// <i>Define modbus connection mode

// <q>Modbus Serial connection
// <i>Enable modbus Serial connection
#define MODBUS_SER_ENABLED              1

// <q>Modbus Ethernet connection
// <i>Enable modbus ethernet connection
#define MODBUS_ETH_ENABLED              0


#if ((MODBUS_SER_ENABLED == 0) && (MODBUS_ETH_ENABLED == 0))
    #error "Please select at least one connection mode!"
#endif

// </h>


#ifdef __cplusplus
}
#endif

#endif

//<<< end of configuration section >>>

