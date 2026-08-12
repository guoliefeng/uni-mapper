cmake_minimum_required(VERSION 3.5)
project(open_lmm_ros VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

if(BUILD_WITH_MARCH_NATIVE)
  add_definitions(-march=native)
  add_definitions(-DBUILD_WITH_MARCH_NATIVE)
  set(CMAKE_C_FLAGS "-march=native ${CMAKE_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "-march=native ${CMAKE_CXX_FLAGS}")
endif()

### ROS version check ###
if(NOT DEFINED ENV{ROS_VERSION})
  message(FATAL_ERROR "ROS_VERSION is not set. Source a ROS1 or ROS2 setup file before building.")
elseif("$ENV{ROS_VERSION}" STREQUAL "1")
  message(STATUS "Current ROS version: ROS1")
  set(ros_version "ros1")

  find_package(catkin REQUIRED COMPONENTS
    open_lmm
    roscpp
    roslib
  )

  catkin_package(
    LIBRARIES open_lmm_ros
    CATKIN_DEPENDS open_lmm roscpp roslib
  )
elseif("$ENV{ROS_VERSION}" STREQUAL "2")
  message(STATUS "Current ROS version: ROS2")
  set(ros_version "ros2")

  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()
else()
  message(FATAL_ERROR "Unsupported ROS_VERSION='$ENV{ROS_VERSION}'. Expected 1 or 2.")
endif()

### open_lmm ###
# catkin_make exposes targets from packages that were configured earlier. Isolated
# catkin and colcon builds do not, so embed the core sources in that case.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../open_lmm" AND
   NOT TARGET open_lmm_map_server)
  set(OPEN_LMM_EMBEDDED_BUILD ON)
  add_subdirectory(
    "${CMAKE_CURRENT_SOURCE_DIR}/../open_lmm"
    "${CMAKE_CURRENT_BINARY_DIR}/open_lmm"
  )
endif()

if("${ros_version}" STREQUAL "ros2")
  ### open_lmm_ros (ROS2 component) ###
  ament_auto_add_library(open_lmm_ros SHARED
    ${ros_version}/open_lmm_ros/open_lmm_ros.cpp
  )
  target_link_libraries(open_lmm_ros
    open_lmm_utils
    open_lmm_map_server
  )
  rclcpp_components_register_nodes(open_lmm_ros "open_lmm::OpenLMMROS")

  ### open_lmm_rosnode ###
  ament_auto_add_executable(open_lmm_rosnode
    ${ros_version}/open_lmm_ros/open_lmm_rosnode.cpp
  )
  target_link_libraries(open_lmm_rosnode open_lmm_ros)

  ament_auto_package()
else()
  ### open_lmm_ros (ROS1 library) ###
  add_library(open_lmm_ros SHARED
    ${ros_version}/open_lmm_ros/open_lmm_ros.cpp
  )
  target_include_directories(open_lmm_ros
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/${ros_version}/open_lmm_ros
    PRIVATE ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(open_lmm_ros
    open_lmm_utils
    open_lmm_map_server
    ${catkin_LIBRARIES}
  )

  ### open_lmm_rosnode ###
  add_executable(open_lmm_rosnode
    ${ros_version}/open_lmm_ros/open_lmm_rosnode.cpp
  )
  target_include_directories(open_lmm_rosnode PRIVATE ${catkin_INCLUDE_DIRS})
  target_link_libraries(open_lmm_rosnode
    open_lmm_ros
    ${catkin_LIBRARIES}
  )

  if(catkin_EXPORTED_TARGETS)
    add_dependencies(open_lmm_ros ${catkin_EXPORTED_TARGETS})
    add_dependencies(open_lmm_rosnode ${catkin_EXPORTED_TARGETS})
  endif()

  install(TARGETS open_lmm_ros open_lmm_rosnode
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  install(DIRECTORY ${ros_version}/launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )
endif()
