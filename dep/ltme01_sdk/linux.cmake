cmake_minimum_required(VERSION 2.8)

add_compile_options(-std=c++11)

set(LIBUSB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/libusb/linux/libusb-1.0.21")
set(SDK_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Sources/include")
set(SDK_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Sources/src")

FILE(GLOB_RECURSE SDK_SRC "${SDK_SRC_DIR}/*.cpp")

include(ExternalProject)
ExternalProject_Add(libusb-1.0.21
  SOURCE_DIR ${LIBUSB_DIR}
  PATCH_COMMAND "bash"
    "-c"
    "! [ -x ${LIBUSB_DIR}/configure ] && chmod +x ${LIBUSB_DIR}/configure || :"
  CONFIGURE_COMMAND "${LIBUSB_DIR}/configure"
    "CFLAGS=-fPIC"
    "--prefix=${CMAKE_CURRENT_BINARY_DIR}/ThirdParty/libusb-1.0.21"
    "--enable-shared=no"
    "--enable-static=yes"
  BUILD_COMMAND "bash"
    "-c"
    "(make check &> /dev/null || touch ${LIBUSB_DIR}/aclocal.m4 ${LIBUSB_DIR}/Makefile.in ${LIBUSB_DIR}/configure ${LIBUSB_DIR}/config.h.in) && make"
  INSTALL_COMMAND make install
)

project(ltme01_sdk)
add_library(${PROJECT_NAME} ${SDK_SRC})
set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "")
add_dependencies(${PROJECT_NAME} libusb-1.0.21)
target_include_directories(${PROJECT_NAME}
  PUBLIC "${SDK_INCLUDE_DIR}/public"
  PRIVATE "${SDK_INCLUDE_DIR}/private"
  PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/ThirdParty/libusb-1.0.21/include"
)
target_link_libraries(${PROJECT_NAME} "-Wl,--exclude-libs,libusb-1.0.a" "${CMAKE_CURRENT_BINARY_DIR}/ThirdParty/libusb-1.0.21/lib/libusb-1.0.a" pthread udev)
