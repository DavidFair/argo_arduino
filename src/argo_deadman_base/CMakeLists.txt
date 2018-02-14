# Include header libraries

link_directories(${PROJECT_SOURCE_DIR}/ext/rfm69)
link_directories(${PROJECT_SOURCE_DIR}/ext/spiflash)

message(test!)
message(${PROJECT_SOURCE_DIR}/ext/spiflash)

# Specify target

if (NOT UNIT_TESTING)
    generate_arduino_firmware(argo_deadman_base
        SRCS argo_deadman_base.cpp
        BOARD ${BOARD_TYPE}
        PORT ${PORT_PATH}
        SERIAL picocom @SERIAL_PORT@ -b ${BAUD_RATE} -l)
endif()