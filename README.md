ROS driver for LitraTech's [LTME-01x series](http://www.litratech.com/products.aspx?leftid=121) 2D laser scanners.

Supported device models:
* LTME-01B (featuring a USB interface)
* LTME-01C (featuring an Ethernet interface, and numerous performance/stability improvements over LTME-01B)

Supported ROS distributions:
* ROS Indigo / Ubuntu 14.04
* ROS Kinetic / Ubuntu 16.04
* ROS Melodic / Ubuntu 18.04

# Build and Install

## Dependencies

This driver requires the following libraries:

* **libudev-dev**: Development files for libudev. Install by `sudo apt install libudev-dev`.

## Build the Package ##

Clone or extract package source to your catkin workspace's `src` directory, then build the workspace:

```
cd ~/catkin_ws/src
git clone https://github.com/LitraTech/ltme01_driver.git
cd .. && catkin_make
```

*Notes for Ninja:*
If you use `ninja` as the build tool for `catkin_make`, please edit `dep/ltme01_sdk/linux.cmake` and add the following line to `ExternalProject_Add` command:
```
BUILD_BYPRODUCTS "${CMAKE_CURRENT_BINARY_DIR}/ThirdParty/libusb-1.0.21/lib/libusb-1.0.a"
```
to prevent `ninja` from reporting dependency errors. This fix is not incorporated into the source tree primarily for compatibility reasons, as `BUILD_BYPRODUCTS` was introduced in CMake 3.x, and Ubuntu 14.04 ships CMake 2.8 by default.

# Driver Setup

## Access USB Device from Non-root Users (LTME-01B Specific)


Under Linux's default system settings, device node for LTME-01B's USB interface is only accessible by *root* user. You'll have to setup a *udev* rule for normal users to access the device. This can be done with:
```
rosrun ltme01_driver create_udev_rules
```
Basically this command will copy a rule file to `/etc/udev/rules.d` and then restart *udev* service. You may need to disconnect and then connect LTME-01B for this to take effect.
