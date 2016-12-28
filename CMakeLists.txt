cmake_minimum_required(VERSION 2.8)
project(msp)

add_definitions(-std=c++11)
add_definitions(-Wall)

# use static boost libraries on Windows
if(WIN32)
    set(Boost_USE_STATIC_LIBS "ON")
endif()

find_package(Boost REQUIRED COMPONENTS system)
include_directories( ${Boost_INCLUDE_DIRS} )

if(MSVC)
    # disable autolinking in boost
    add_definitions(-DBOOST_ALL_NO_LIB)
endif()

if(UNIX)
    set(BUILD_SHARED_LIBS "ON")
endif()

find_package(Threads)

find_package(catkin QUIET)

if(catkin_FOUND)
    catkin_package(
        INCLUDE_DIRS inc
        LIBRARIES msp msp_fcu msp_msg_print
    #    DEPENDS Boost
    )
endif()

set(MSP_SOURCE_DIR src)

set(MSP_INCLUDE_DIR inc/msp)

include_directories(${MSP_INCLUDE_DIR})

set(MSP_SRC
    ${MSP_SOURCE_DIR}/SerialPort.cpp
    ${MSP_SOURCE_DIR}/MSP.cpp
    ${MSP_SOURCE_DIR}/deserialise.cpp
)

set(MSP_HDR
    ${MSP_INCLUDE_DIR}/msp_msg.hpp
    ${MSP_INCLUDE_DIR}/msp_id.hpp
    ${MSP_INCLUDE_DIR}/types.hpp
    ${MSP_INCLUDE_DIR}/SerialPort.hpp
    ${MSP_INCLUDE_DIR}/MSP.hpp
    ${MSP_INCLUDE_DIR}/deserialise.hpp
)

# low-level API
add_library(msp ${MSP_SRC} ${MSP_HDR})
target_link_libraries(msp ${Boost_SYSTEM_LIBRARIES})
target_link_libraries(msp ${CMAKE_THREAD_LIBS_INIT})

# printing message content
add_library(msp_msg_print ${MSP_SOURCE_DIR}/msg_print.cpp
                                 ${MSP_INCLUDE_DIR}/msg_print.hpp)

# test reading speed
add_executable(get_msp_info examples/msp_connection_test.cpp)
target_link_libraries(get_msp_info msp)

# test and print all requests
add_executable(msp_read_test examples/msp_read_test.cpp)
target_link_libraries(msp_read_test msp msp_msg_print)

# high-level API
add_library(msp_fcu ${MSP_SOURCE_DIR}/FlightController.cpp
                           ${MSP_INCLUDE_DIR}/FlightController.hpp
                           ${MSP_SOURCE_DIR}/PeriodicTimer.cpp
                           ${MSP_INCLUDE_DIR}/PeriodicTimer.hpp)
target_link_libraries(msp_fcu msp)

# testing publish/subscribe
add_executable(fcu_test examples/fcu_test.cpp)
target_link_libraries(fcu_test msp_fcu msp_msg_print)

# test arming and disarming
add_executable(fcu_arm examples/fcu_arm_test.cpp)
target_link_libraries(fcu_arm msp_fcu)

# test setting motors directly
add_executable(fcu_motors examples/fcu_motor_test.cpp)
target_link_libraries(fcu_motors msp_fcu)

# subscribing with custom type
add_executable(fcu_custom_type examples/fcu_custom_type.cpp)
target_link_libraries(fcu_custom_type msp_fcu)

# cleanflight requests
add_executable(cleanflight_read_test examples/cleanflight_read_test.cpp)
target_link_libraries(cleanflight_read_test msp msp_msg_print)

# installation
if(catkin_FOUND)
    install(TARGETS msp msp_fcu msp_msg_print
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

    install(DIRECTORY inc/${PROJECT_NAME}/
       DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
       FILES_MATCHING PATTERN "*.hpp"
    )
else()
    install(TARGETS msp
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
    install(TARGETS msp_msg_print
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
    install(TARGETS msp_fcu
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
    install(DIRECTORY ${MSP_INCLUDE_DIR} DESTINATION include/ FILES_MATCHING PATTERN "*.hpp")

    SET(PKG_CONFIG_LIBDIR       "\${prefix}/lib" )
    SET(PKG_CONFIG_INCLUDEDIR   "\${prefix}/include/msp" )
    SET(PKG_CONFIG_LIBS         "-L\${libdir} -lmsp -lmsp_fcu -lboost_system -lmsp_msg_print" )
    SET(PKG_CONFIG_CFLAGS       "-I\${includedir}" )

    CONFIGURE_FILE(
      "${CMAKE_CURRENT_SOURCE_DIR}/pkg-config.pc.cmake"
      "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.pc"
    )

    INSTALL(FILES "${CMAKE_BINARY_DIR}/${PROJECT_NAME}.pc"
            DESTINATION lib/pkgconfig)
endif()

#get_cmake_property(_variableNames VARIABLES)
#foreach (_variableName ${_variableNames})
#    message(STATUS "${_variableName}=${${_variableName}}")
#endforeach()
