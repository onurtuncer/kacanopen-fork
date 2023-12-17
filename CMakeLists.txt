cmake_minimum_required(VERSION 2.8)
project(kacanopen)

#############
##  Flags  ##
#############
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

################
##  Settings  ##
################
# Debug
option(EXHAUSTIVE_DEBUGGING "Enable exhaustive debugging." OFF)
if(${EXHAUSTIVE_DEBUGGING})
  message(STATUS "Enabled exhaustive debugging.")
  add_definitions("-DEXHAUSTIVE_DEBUGGING")
endif()

# ##############
# ##  Catkin  ##
# ##############
# find_package(catkin REQUIRED COMPONENTS
#   backward_ros
#   roscpp
#   roslib
#   sensor_msgs
#   std_msgs
# )

find_package(Threads)
find_package(Boost 1.46.1 COMPONENTS system filesystem REQUIRED)
# catkin_package(
#   INCLUDE_DIRS
#     include
#   LIBRARIES
#     ${PROJECT_NAME}_driver
#     ${PROJECT_NAME}_core
#     ${PROJECT_NAME}_master
#     ${PROJECT_NAME}_ros_bridge
#     ${PROJECT_NAME}_tools
#   CATKIN_DEPENDS
#     backward_ros
#     roscpp
#     roslib
#     sensor_msgs
#     std_msgs
# )

#############
##  Build  ##
#############
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  # ${catkin_INCLUDE_DIRS}
)

# Driver
file(GLOB DRIVER_SRC "src/driver/*.c")
add_library(${PROJECT_NAME}_driver SHARED ${DRIVER_SRC})

# Core
file(GLOB CORE_SRC "src/core/*.cpp")
add_library(${PROJECT_NAME}_core SHARED ${CORE_SRC})
target_link_libraries(${PROJECT_NAME}_core
  ${PROJECT_NAME}_driver
  ${CMAKE_DL_LIBS}
  ${CMAKE_THREAD_LIBS_INIT}
)

# Master
file(GLOB MASTER_SRC "src/master/*.cpp")
add_library(${PROJECT_NAME}_master SHARED ${MASTER_SRC})
target_link_libraries(${PROJECT_NAME}_master
  ${Boost_SYSTEM_LIBRARY}
  ${Boost_FILESYSTEM_LIBRARY}
  ${CMAKE_THREAD_LIBS_INIT}
  ${PROJECT_NAME}_core
)

# # Ros bridge
# file(GLOB ROS_BRIDGE_SRC "src/ros_bridge/*.cpp")
# add_library(${PROJECT_NAME}_ros_bridge SHARED ${ROS_BRIDGE_SRC})
# target_link_libraries(${PROJECT_NAME}_ros_bridge
#   ${PROJECT_NAME}_master
#   ${catkin_LIBRARIES}
# )

# Tools
file(GLOB TOOLS_SRC "src/tools/*.cpp")
add_library(${PROJECT_NAME}_tools SHARED ${TOOLS_SRC})
target_link_libraries(${PROJECT_NAME}_tools
  ${CMAKE_DL_LIBS}
  ${CMAKE_THREAD_LIBS_INIT}
)

##############
## Examples ##
##############
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_subdirectory(examples)
else()
  message("\n-- Add -DCMAKE_BUILD_TYPE=Debug to build the examples.\n")
endif()

# #############
# ## Install ##
# #############
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )

install(
  TARGETS
    ${PROJECT_NAME}_driver
    ${PROJECT_NAME}_core
    ${PROJECT_NAME}_master
    # ${PROJECT_NAME}_ros_bridge
    ${PROJECT_NAME}_tools
  # ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  # LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  # RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# install(
#   DIRECTORY resources
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

# file(GLOB SCRIPTS "scripts/*")
# install(
#   PROGRAMS ${SCRIPTS}
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
