cmake_minimum_required(VERSION 2.8)
project(msp)

add_definitions(-std=c++11)

find_package(Boost REQUIRED system)

set(MSP_SOURCE_DIR src)

set(MSP_SRC
    ${MSP_SOURCE_DIR}/SerialPort.cpp
    ${MSP_SOURCE_DIR}/MSP.cpp
)

set(MSP_HDR
    ${MSP_SOURCE_DIR}/msp_msg.h
    ${MSP_SOURCE_DIR}/SerialPort.hpp
    ${MSP_SOURCE_DIR}/MSP.hpp
)

include_directories(${MSP_SOURCE_DIR})

add_library(msp ${MSP_SRC})
target_link_libraries(msp ${Boost_SYSTEM_LIBRARIES})

add_executable(get_msp_info main.cpp)
target_link_libraries(get_msp_info msp)

#get_cmake_property(_variableNames VARIABLES)
#foreach (_variableName ${_variableNames})
#    message(STATUS "${_variableName}=${${_variableName}}")
#endforeach()
