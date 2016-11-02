cmake_minimum_required(VERSION 2.8)
project(msp)

add_definitions(-std=c++11)

find_package(Boost REQUIRED system)
find_package(Threads)

set(MSP_SOURCE_DIR src)

set(MSP_SRC
    ${MSP_SOURCE_DIR}/SerialPort.cpp
    ${MSP_SOURCE_DIR}/MSP.cpp
    ${MSP_SOURCE_DIR}/deserialise.cpp
)

set(MSP_HDR
    ${MSP_SOURCE_DIR}/msp_msg.hpp
    ${MSP_SOURCE_DIR}/msp_id.hpp
    ${MSP_SOURCE_DIR}/types.hpp
    ${MSP_SOURCE_DIR}/SerialPort.hpp
    ${MSP_SOURCE_DIR}/MSP.hpp
    ${MSP_SOURCE_DIR}/deserialise.hpp
)

include_directories(${MSP_SOURCE_DIR})

add_library(msp ${MSP_SRC} ${MSP_HDR})
target_link_libraries(msp ${Boost_SYSTEM_LIBRARIES})
target_link_libraries(msp ${CMAKE_THREAD_LIBS_INIT})

add_executable(get_msp_info main.cpp)
target_link_libraries(get_msp_info msp)

add_library(msp_fcu ${MSP_SOURCE_DIR}/FlightController.cpp)
target_link_libraries(msp_fcu msp)

add_executable(fc_test fc_test.cpp)
target_link_libraries(fc_test msp_fcu)

#get_cmake_property(_variableNames VARIABLES)
#foreach (_variableName ${_variableNames})
#    message(STATUS "${_variableName}=${${_variableName}}")
#endforeach()
