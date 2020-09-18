cmake_minimum_required(VERSION 3.10.2)
project(pool_allocator_demo)

# C++17
set(CMAKE_CXX_STANDARD 17)

# System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
)

###################################
## Catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
  roscpp
  sensor_msgs
  std_msgs
  LIBRARIES
    ${PROJECT_NAME}
  DEPENDS
    Boost
)

###########
## Build ##
###########

# Specify additional locations of header files
# Your package locations should be listed before other locations
include_directories(
  SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
)

add_executable(
  ${PROJECT_NAME}
  src/pool_allocator_demo.cpp
)
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(
  default_allocator_demo
  src/default_allocator_demo.cpp
)
target_link_libraries(
  default_allocator_demo
  ${catkin_LIBRARIES}
)

add_executable(
  pool_allocator_image_demo
  src/pool_allocator_image_demo.cpp
)
target_link_libraries(
  pool_allocator_image_demo
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS
    ${PROJECT_NAME}
    pool_allocator_image_demo
    default_allocator_demo
  ARCHIVE DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)
