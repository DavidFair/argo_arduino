# Argo Control Software

# Table of Contents
  1. [Requirements](#Requirements)
  2. [Building Arduino Firmware](#building-arduino-firmware)  
    2.1. [Target Names](#target-names)
  3. [Uploading and Communications](#uploading-and-communications)  
    3.1 [Getting access to TTY device](#getting-access-to-tty-device)  
    3.2 [Exiting picocom](#exitting-picocom)



## Requirements

The following dependencies are required:

- arduino
- picocom
- cmake

The following are required but installed by the Arduino SDK
- gcc-avr 
- binutils-avr 
- avr-libc 
- avrdude 


These can be installed with the following command

```sh
sudo apt-get install arduino cmake gcc-avr binutils-avr avr-libc avrdude picocom
```


## Building Arduino Firmware

1. Clone the repository - for example into `argo_src`
2. Create a build folder - for example `argo_build`
3. `cd` into the build folder
4. Run `cmake ../<your src folder name>`
5. Run `make` to build the firmware

For example:

```sh
git clone https://github.com/DavidFair/argo_arduino argo_src
mkdir argo_build
cd argo_build
cmake ../argo_src
make -j 4
```

### Arduino Target Names

The following targets build firmware for the Arduino:

- `argo_rc`
- `argo_deadman_base`
- `argo_deadman_remote`

## Uploading and Communications

From the build folder targets to upload and open serial communications the following make targets need to be built:

- `<target_name>-upload` - To upload firmware
- `<target_name>-serial` - To communicate over serial

For example to upload the `argo_deadman_base` firmware the following would be used

```sh
make argo_deadman_base-upload
```

To communicate with the Arduino the following opens a serial console

```sh
make argo_rc-serial
```

### Getting access to TTY device

On Linux the user will needed to be added to the `dialout` group to access the TTY device without sudo to upload and use the serial console.

This is **strongly** recommended to do rather than running make with root permissions and only needs to be completed once.

Add the current user to the dialout group:

```sh
sudo usermod -a -G dialout $USER
```

The user may have to log out and in to join the new group.

### Exiting Picocom

To exit picocom the following keys are used

- CTRL-A
- CTRL-X

*Note: CTRL-C will be sent to the device and appear to do nothing*