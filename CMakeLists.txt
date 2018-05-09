cmake_minimum_required(VERSION 2.8.3)
project(ltme01_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package()

###########
## Build ##
###########

set(LTME01_SDK_DIR "dep/ltme01_sdk")

set(BUILD_SHARED_LIBS OFF)

add_subdirectory(${LTME01_SDK_DIR})

include_directories(
  "${LTME01_SDK_DIR}/Sources/include"
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)

add_executable(${PROJECT_NAME}_node
  src/device_notifier.cpp
  src/lidar_driver.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ltme01_sdk
)

#############
## Install ##
#############

install(PROGRAMS
  scripts/create_udev_rules
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  ltme01.rules
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
