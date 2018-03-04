# External libraries

# Encoder
set(ENCODER_SRC
  encoder/Encoder.cpp)

set(ENCODER_HDR
  encoder/Encoder.h
  encoder/utility/direct_pin_read.h
  encoder/utility/interrupt_config.h
  encoder/utility/interrupt_pins.h)

generate_arduino_library(
    encoder
    SRCS ${ENCODER_SRC}
    HDRS ${ENCODER_HDR}
    BOARD mega2560
)
