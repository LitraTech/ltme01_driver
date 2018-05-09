cmake_minimum_required(VERSION 2.8)

if(MINGW)
  add_compile_options(-std=c++11)
endif()

set(LIBUSB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/libusb/win32/libusb-1.0.21")
if(MSVC)
  set(LIBUSB_LIBRARY "${LIBUSB_DIR}/MS32/static/libusb-1.0.lib")
elseif(MINGW)
  set(LIBUSB_LIBRARY "${LIBUSB_DIR}/MinGW32/static/libusb-1.0.a")
endif()

set(SDK_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Sources/include")
set(SDK_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Sources/src")

FILE(GLOB_RECURSE SDK_SRC "${SDK_SRC_DIR}/*.cpp")

project(ltme01_sdk)
add_library(${PROJECT_NAME} ${SDK_SRC})
set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "")
set_target_properties(${PROJECT_NAME} PROPERTIES IMPORT_PREFIX "")
if(BUILD_SHARED_LIBS)
  target_compile_definitions(${PROJECT_NAME} PRIVATE LTME01_SDK_EXPORTS)
else()
  target_compile_definitions(${PROJECT_NAME} PRIVATE LTME01_SDK_STATIC)
endif()
target_include_directories(${PROJECT_NAME}
  PUBLIC "${SDK_INCLUDE_DIR}/public"
  PRIVATE "${SDK_INCLUDE_DIR}/private"
  PRIVATE "${LIBUSB_DIR}/include")
target_link_libraries(${PROJECT_NAME} ${LIBUSB_LIBRARY})
