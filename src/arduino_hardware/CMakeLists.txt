# This folder only contains targets used by the actual Arduino hardware
if (NOT UNIT_TESTING)

set(SRC_FILES
    ArduinoHardware.cpp
)

set(HDR_FILES
    ArduinoHardware.hpp
)

include_directories(
    "${PROJECT_SOURCE_DIR}/src/arduino_interface"
    "${PROJECT_SOURCE_DIR}/src/lib_wrappers"
)

generate_arduino_library(
  ArduinoHardware
  LIBS ArduinoInterface
  SRCS ${SRC_FILES}
  HDRS ${HDR_FILES}
  BOARD ${BOARD_TYPE}
)
endif()