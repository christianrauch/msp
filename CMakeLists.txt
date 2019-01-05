cmake_minimum_required(VERSION 3.1)
project(msp)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(UNIX)
    set(BUILD_SHARED_LIBS "ON")
endif()

add_definitions(-DASIO_STANDALONE)

if(WIN32)
    include_directories(${ASIO_HEADER_PATH})
endif()

OPTION(BUILD_EXAMPLES "Build Library with examples" OFF)
OPTION(BUILD_TESTS "Build Library with tests" OFF)

find_package(Threads)

set(MSP_SOURCE_DIR src)
set(MSP_INCLUDE_DIR inc/msp)
include_directories(${MSP_INCLUDE_DIR})


################################################################################
### libraries

# client library
add_library(mspclient ${MSP_SOURCE_DIR}/Client.cpp ${MSP_SOURCE_DIR}/periodic_timer.cpp)
target_link_libraries(mspclient ${CMAKE_THREAD_LIBS_INIT})

# high-level API
add_library(msp_fcu ${MSP_SOURCE_DIR}/FlightController.cpp)
target_link_libraries(msp_fcu mspclient)


################################################################################
### examples / tests

if(BUILD_EXAMPLES)
       
    # testing publish/subscribe
    add_executable(fcu_test examples/fcu_test.cpp)
    target_link_libraries(fcu_test msp_fcu)

    # test arming and disarming
    add_executable(fcu_arm examples/fcu_arm_test.cpp)
    target_link_libraries(fcu_arm msp_fcu)

    # test setting motors directly
    add_executable(fcu_motors examples/fcu_motor_test.cpp)
    target_link_libraries(fcu_motors msp_fcu)

    # subscribing with custom type
    add_executable(fcu_custom_type examples/fcu_custom_type.cpp)
    target_link_libraries(fcu_custom_type msp_fcu)

    # client test for asynchronous callbacks
    add_executable(client_async_test examples/client_async_test.cpp)
    target_link_libraries(client_async_test mspclient )

    # client test for blocking read
    add_executable(client_read_test examples/client_read_test.cpp)
    target_link_libraries(client_read_test mspclient )

endif()

################################################################################
### installation

#install(TARGETS msp msp_msg_print msp_fcu mspclient
install(TARGETS msp_fcu mspclient
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)
install(DIRECTORY ${MSP_INCLUDE_DIR} DESTINATION include/ FILES_MATCHING PATTERN "*.hpp")

SET(PKG_CONFIG_LIBDIR       "\${prefix}/lib" )
SET(PKG_CONFIG_INCLUDEDIR   "\${prefix}/include/" )
SET(PKG_CONFIG_LIBS         "-L\${libdir} -lmsp_fcu -lmspclient" )
SET(PKG_CONFIG_CFLAGS       "-I\${includedir}" )

CONFIGURE_FILE(
  "${CMAKE_CURRENT_SOURCE_DIR}/pkg-config.pc.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.pc"
)

INSTALL(FILES "${CMAKE_BINARY_DIR}/${PROJECT_NAME}.pc"
        DESTINATION lib/pkgconfig)



###############################################################################
### testing
if(BUILD_TESTS)
    enable_testing()

    # Download and unpack googletest at configure time
    configure_file(CMakeLists.txt.in googletest/CMakeLists.txt)
    execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
      RESULT_VARIABLE result
      WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest )
    if(result)
      message(FATAL_ERROR "CMake step for googletest failed: ${result}")
    endif()
    execute_process(COMMAND ${CMAKE_COMMAND} --build .
      RESULT_VARIABLE result
      WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest )
    if(result)
      message(FATAL_ERROR "Build step for googletest failed: ${result}")
    endif()

    # Prevent overriding the parent project's compiler/linker
    # settings on Windows
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

    # Add googletest directly to our build. This defines
    # the gtest and gtest_main targets.
    add_subdirectory(/usr/src/gtest
                     ${CMAKE_BINARY_DIR}/googletest-build
                     EXCLUDE_FROM_ALL)

    # The gtest/gtest_main targets carry header search path
    # dependencies automatically when using CMake 2.8.11 or
    # later. Otherwise we have to add them here ourselves.
    if (CMAKE_VERSION VERSION_LESS 2.8.11)
      include_directories("${gtest_SOURCE_DIR}/include")
    endif()

    add_executable(value_test test/value_test.cpp)
    target_link_libraries(value_test gtest_main)
    add_test(NAME value_test COMMAND value_test)

    add_executable(bytevector_test test/ByteVector_test.cpp)
    target_link_libraries(bytevector_test gtest_main)
    add_test(NAME bytevector_test COMMAND bytevector_test)

    add_executable(msp_msg_test test/msp_msg_test.cpp)
    target_link_libraries(msp_msg_test gtest_main)
    add_test(NAME msp_msg_test COMMAND msp_msg_test)

endif()
