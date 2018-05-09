cmake_minimum_required(VERSION 2.8)

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  include("linux.cmake")
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  include("win32.cmake")
endif()
