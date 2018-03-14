# This folder only contains targets used by the actual Arduino hardware
if (NOT UNIT_TESTING)

set(SRC_FILES
    arduino_hardware.cpp 
)

set(HDR_FILES
    arduino_hardware.hpp
)

include_directories(
    "${PROJECT_SOURCE_DIR}/src/arduino_interface"
    "${PROJECT_SOURCE_DIR}/src/lib_wrappers"
)

generate_arduino_library(
  arduino_hardware
  LIBS arduino_interface
  SRCS ${SRC_FILES}
  HDRS ${HDR_FILES}
  BOARD ${BOARD_TYPE}
)
endif()