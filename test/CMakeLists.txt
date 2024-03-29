# Adapted from https://github.com/google/googletest/blob/master/googletest/README.md

# Download and unpack googletest at configure time
configure_file(CMakeLists.txt.in ${CMAKE_BINARY_DIR}/googletest-download/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest-download )

if(result)
  message(FATAL_ERROR "CMake step for googletest failed: ${result}")
endif()

execute_process(COMMAND ${CMAKE_COMMAND} --build .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest-download )

if(result)
  message(FATAL_ERROR "Build step for googletest failed: ${result}")
endif()

# Add googletest directly to our build. This defines
# the gtest and gtest_main targets.
add_subdirectory(${CMAKE_BINARY_DIR}/googletest-src
                 ${CMAKE_BINARY_DIR}/googletest-build
                 EXCLUDE_FROM_ALL)


#### Define our test targets #####

# Sanity check

add_executable(example example.cpp)
target_link_libraries(example gtest_main)
add_test(NAME example_test COMMAND example)

# Actual unit tests

include_directories(
    "${PROJECT_SOURCE_DIR}/src/arduino_interface"
    "${PROJECT_SOURCE_DIR}/src/argo_rc"
    "${PROJECT_SOURCE_DIR}/src/lib_wrappers"
    )

add_library(test_main STATIC main.cpp)
target_link_libraries(test_main gmock_main)

add_executable(argo_rc_test argo_rc_lib_test.cpp)
add_executable(EncoderTest EncoderTest.cpp)
add_executable(SerialComms_test SerialCommsTest.cpp)
add_executable(PidControllerTest PidControllerTest.cpp)

set(REQUIRED_LIBS
  test_main
  gmock_main
  argo_lib
  ArduinoInterface
  lib_wrappers
)

target_link_libraries(argo_rc_test ${REQUIRED_LIBS})
target_link_libraries(EncoderTest ${REQUIRED_LIBS})
target_link_libraries(SerialComms_test ${REQUIRED_LIBS})
target_link_libraries(PidControllerTest ${REQUIRED_LIBS})

add_test(NAME argo_rc_test COMMAND argo_rc_test)
add_test(NAME EncoderTest COMMAND EncoderTest)
add_test(NAME SerialComms_test COMMAND SerialComms_test)
add_test(NAME PidControllerTest COMMAND PidControllerTest)