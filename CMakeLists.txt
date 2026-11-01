cmake_minimum_required(VERSION 3.16)
project(sapphire_ros1)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

option(SAPPHIRE_ENABLE_LTO
  "Enable IPO/LTO for Sapphire Release targets when supported" ON)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  tf2
  tf2_ros
  visualization_msgs
)
find_package(Eigen3 REQUIRED)

# The adapter itself does not link PCL. The core does, and Noetic's system
# PCL 1.10 must never be selected in place of the pinned 1.14 installation.
find_package(PCL 1.14 REQUIRED)
if(PCL_VERSION VERSION_LESS 1.14.0)
  message(FATAL_ERROR
    "Sapphire ROS 1 requires PCL >= 1.14.0; found ${PCL_VERSION} at ${PCL_DIR}")
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../sapphire/CMakeLists.txt")
  message(STATUS "sapphire core found as sibling package, using add_subdirectory")
  add_subdirectory(
    "${CMAKE_CURRENT_SOURCE_DIR}/../sapphire"
    "${CMAKE_BINARY_DIR}/sapphire"
  )
else()
  message(FATAL_ERROR
    "Sapphire ROS 1 expects the core at ../sapphire/CMakeLists.txt")
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sapphire_ros1_adapter
  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    roscpp
    sensor_msgs
    tf2
    tf2_ros
    visualization_msgs
)

add_library(sapphire_ros1_adapter
  src/conversion.cpp
  src/sapphire_ros.cpp
)
target_include_directories(sapphire_ros1_adapter PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(sapphire_ros1_adapter PUBLIC
  sapphire
  ${catkin_LIBRARIES}
)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU"
   AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9.1)
  target_link_libraries(sapphire_ros1_adapter PUBLIC stdc++fs)
endif()

if(SAPPHIRE_ENABLE_LTO)
  include(CheckIPOSupported)
  check_ipo_supported(
    RESULT SAPPHIRE_ROS1_IPO_SUPPORTED
    OUTPUT SAPPHIRE_ROS1_IPO_ERROR
    LANGUAGES CXX
  )
  if(SAPPHIRE_ROS1_IPO_SUPPORTED)
    set_property(TARGET sapphire_ros1_adapter
      PROPERTY INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE)
  else()
    message(WARNING
      "SAPPHIRE_ENABLE_LTO is ON, but IPO/LTO is unavailable: "
      "${SAPPHIRE_ROS1_IPO_ERROR}")
  endif()
endif()

add_executable(sapphire_ros_node src/main.cpp)
target_link_libraries(sapphire_ros_node sapphire_ros1_adapter)

if(SAPPHIRE_ENABLE_LTO AND SAPPHIRE_ROS1_IPO_SUPPORTED)
  set_property(TARGET sapphire_ros_node
    PROPERTY INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE)
endif()

install(
  TARGETS sapphire_ros_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
  TARGETS sapphire_ros1_adapter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(sapphire_ros1_conversion_test
    test/conversion_test.cpp
  )
  if(TARGET sapphire_ros1_conversion_test)
    target_link_libraries(
      sapphire_ros1_conversion_test
      sapphire_ros1_adapter
      gtest_main
    )
  endif()
endif()
