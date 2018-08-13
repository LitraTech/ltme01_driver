ROS driver for LitraTech's [LTME-01 2D laser scanner](http://www.litratech.com/products.aspx?leftid=121), based on the [official C++ SDK](https://github.com/LitraTech/ltme01_sdk). It has been tested to work under the following combinations:
* ROS Indigo + Ubuntu 14.04
* ROS Kinetic + Ubuntu 16.04

# Build and Install

## Dependencies

This driver depends on the following libraries:

* **LTME-01 SDK**: The SDK, as well as its dependencies (libusb), has been incorporated into the source tree so you don't need to install them manually.
* **libudev-dev**: Development files for libudev library.

## Build Instruction

Build this driver as you would normally do for any other ROS packages: change directory to the ROS workspace where driver source resides in and run the following command:
```
catkin_make
```

## Notes for Ninja

`catkin_make` supports using an alternative build system other than plain `make`. If you choose `ninja` as the underlying tool to build the ROS workspace, please edit the file `linux.cmake` under `ltme01_driver/dep/ltme01_sdk` and add the following option to `ExternalProject_Add` function invocation there:
```
BUILD_BYPRODUCTS "${CMAKE_CURRENT_BINARY_DIR}/ThirdParty/libusb-1.0.21/lib/libusb-1.0.a"
```
This option is not required by other build systems, and we choose not to include it in the source, since `BUILD_BYPRODUCTS` was introduced in CMake 3.x, and we'd like to maintain compatibility with Ubuntu 14.04, which ships CMake 2.8 by default.

# Driver Setup

## Accessing Scan Data from Non-root Users

LTME-01 sends out scan data through USB interface, which is only accessible by *root* under Linux's default settings. To work around this, a *udev* rule has to be setup properly so that ROS nodes launched by an ordinary user may read laser scans without escalated privilege. This can be achieved by executing the following command:
```
rosrun ltme01_driver create_udev_rules
```
Basically this script will copy a rule file to `/etc/udev/rules.d` and then restart the *udev* subsystem. You may need to disconnect and then connect the LTME-01 device for this to take effect.
