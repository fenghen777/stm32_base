## 指定最低版本
cmake_minimum_required(VERSION 3.5.1)

set(MODBUS_PATH ${CMAKE_CURRENT_SOURCE_DIR}/..)

## 搜索源文件
aux_source_directory(./ SRC_MODBUS)
include_directories(${MODBUS_PATH}/inc)
include_directories(${MODBUS_PATH}/port/linux)


## modbus.so
set(MODBUS_LIB modbussrc)
# set(LIBRARY_OUTPUT_PATH ${LIBPATH})
add_library("${MODBUS_LIB}" STATIC ${SRC_MODBUS})
set_target_properties("${MODBUS_LIB}" PROPERTIES OUTPUT_NAME "${MODBUS_LIB}")