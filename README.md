# Argo Control Software

# Table of Contents
* 1\. [Requirements](#Requirements)
* 2\. [Building Arduino Firmware](#building-arduino-firmware)
  * 2.1\. [Target Names](#target-names)
* 3\. [Uploading and Communications](#uploading-and-communications)
  * 3.1\. [Getting access to TTY device](#getting-access-to-tty-device)
  * 3.2\. [(Alternative) Using screen](#\(alternative\)-using-screen)
  * 3.2\. [Exiting picocom](#exitting-picocom)
* 4\. [Running Unit Tests](#running-unit-tests)


## Requirements

The following dependencies are required:

- arduino
- cmake
- git
- picocom

The following are required but installed by the Arduino SDK
- gcc-avr
- binutils-avr
- avr-libc
- avrdude


These can be installed with the following command

```sh
sudo apt-get install arduino cmake gcc-avr binutils-avr avr-libc avrdude picocom git
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

The following target builds firmware for the Arduino:

- `argo_rc`

## Uploading and Communications

From the build folder targets to upload and open serial communications the following make targets need to be built:

- `<target_name>-upload` - To upload firmware
- `<target_name>-serial` - To communicate over serial

For example to upload the `argo_rc` firmware the following would be used

```sh
make argo_rc-upload
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

### (Alternative) Using Screen
Alternatively users can use screen to have read-only communications with
the Arduino.

To use screen the format is as follows:

*screen </dev/ttyPath> \<baudRate>*

Where /dev/ttyPath and baudRate are replaced with appropriate values. For example:

```sh
sudo apt-get install screen

screen /dev/ttyACM0 115200
```

To exit screen the command is **Ctrl+A -> Shift+K**

### Exiting Picocom

To exit picocom the following keys are used

- CTRL-A
- CTRL-X

*Note: CTRL-C will be sent to the device and appear to do nothing*

## Running Unit Tests

A separate build folder is required to build unit tests. Folders
cannot switch between the AVR and X86 compilers without first deleting
the CMake cache.

1. Clone the repository - for example into `argo_src`
2. Create a build folder - for example `argo_tests`
3. `cd` into the build folder
4. Run `cmake ../<your src folder name> -DUNIT_TESTING=ON`
5. Run `make` to compile the unit tests
6. Run `ctest` in to run the unit tests *OR*
7. Run `ctest -V` the see individual tests completed


For example:

```sh
git clone https://github.com/DavidFair/argo_arduino argo_src
mkdir argo_tests
cd argo_tests
cmake ../argo_src -DUNIT_TESTING=ON
make -j4
ctest -V
```

A single set of tests can be run instead using the `-R` flag. For example

```sh
ctest -R SerialComms_test
```