cmake_minimum_required(VERSION 2.8)
project(kacanopen)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

################
##  Settings  ##
################
include(CMakeSettings.txt)

##############
##  Driver  ##
##############
if (DEFINED CAN_DRIVER_NAME)
	message(WARNING "CAN_DRIVER_NAME is deprecated. Use DRIVER instead. Overwriting DRIVER with CAN_DRIVER_NAME (${CAN_DRIVER_NAME}) now...")
	set(DRIVER "${CAN_DRIVER_NAME}")
endif()

foreach(BUILD_DRIVER ${BUILD_DRIVERS})
	# TODO: Replace with CMake 3.3 command IN_LIST
	list(FIND SUPPORTED_DRIVERS "${BUILD_DRIVER}" supported)
	if (${supported} EQUAL -1)
		message(FATAL_ERROR "Driver ${BUILD_DRIVER} does not exist, but you've specified it in BUILD_DRIVERS.")
	endif()
endforeach()

list(FIND SUPPORTED_DRIVERS "${DRIVER}" driver_supported)
if (${driver_supported} EQUAL -1)
	message(FATAL_ERROR "Driver ${DRIVER} does not exist, but you've set it as DRIVER.")
endif()

set(DRIVER_LINK_NAME can_${DRIVER})
message(STATUS "Using CAN driver ${DRIVER_LINK_NAME}")

if(${BUILD_ALL_DRIVERS})
	message(STATUS "Building all available drivers.")
endif()


###############
##  Release  ##
###############
if(${CMAKE_BUILD_TYPE} MATCHES "Release")
	message(STATUS "You are in release mode.")
else()
	if(${EXHAUSTIVE_DEBUGGING})
		message(STATUS "Enabled exhaustive debugging.")
		add_definitions("-DEXHAUSTIVE_DEBUGGING")
	else()
		message(STATUS "Enabled debugging.")
	endif()
endif()


#################
##  Constants  ##
#################
add_definitions("-DSDO_RESPONSE_TIMEOUT_MS=${SDO_RESPONSE_TIMEOUT_MS}")
message(STATUS "SDO response timeout is set to ${SDO_RESPONSE_TIMEOUT_MS}ms")

add_definitions("-DBUSNAME=\"${BUSNAME}\"")
message(STATUS "Busname is set to ${BUSNAME} (only used by examples)")

add_definitions("-DBAUDRATE=${BAUDRATE}")
message(STATUS "Baudrate is set to ${BAUDRATE} (only used by examples)")

add_definitions("-DCONSECUTIVE_SEND_PAUSE_MS=${CONSECUTIVE_SEND_PAUSE_MS}")
message(STATUS "Pause between two consecutively sent CAN frames is set to ${CONSECUTIVE_SEND_PAUSE_MS}ms")


####################
##  Dependencies  ##
####################
find_package(Threads)
find_package(Boost 1.46.1 COMPONENTS system filesystem REQUIRED) 

# Boost required for property_tree / ini_parser
if(NOT(Boost_FOUND))
	message(FATAL_ERROR "Boost > 1.46.1 not found. Boost is required for ini_parser class. This is a header-only library. You can simply download the latest boost package from http://www.boost.org/, extract it into any directory and include it using the following catkin/CMake argument: -DBOOST_ROOT=\"/path/to/boost/\"")
endif()

message(STATUS "Found Boost headers in ${Boost_INCLUDE_DIRS}")
message(STATUS "Found Boost system library: ${Boost_SYSTEM_LIBRARY}")
message(STATUS "Found Boost filesystem library: ${Boost_FILESYSTEM_LIBRARY}")
message(STATUS "Found Threads library: ${CMAKE_THREAD_LIBS_INIT}")


##############
##  Catkin  ##
##############
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${CAN_DRIVER}
    kacanopen_core
    kacanopen_master
    kacanopen_ros_bridge
    kacanopen_tools
  CATKIN_DEPENDS
    message_runtime
)

#############
##  Build  ##
#############
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# Core
file(GLOB CORE_SRC "src/core/*.cpp")
add_library(kacanopen_core SHARED ${CORE_SRC})
target_link_libraries(${TARGET_NAME}
  ${DRIVER_LINK_NAME}
  ${CMAKE_DL_LIBS}
  ${CMAKE_THREAD_LIBS_INIT}
)

# Master
file(GLOB MASTER_SRC "src/master/*.cpp")
add_library(kacanopen_master SHARED ${MASTER_SRC})
target_link_libraries(${TARGET_NAME}
  ${Boost_SYSTEM_LIBRARY}
  ${Boost_FILESYSTEM_LIBRARY}
  ${CMAKE_THREAD_LIBS_INIT}
  kacanopen_core
)

# Ros bridge
file(GLOB ROS_BRIDGE_SRC "src/ros_bridge/*.cpp")
add_library(kacanopen_ros_bridge SHARED ${ROS_BRIDGE_SRC})
target_link_libraries(${TARGET_NAME}
  kacanopen_master
  ${catkin_LIBRARIES}
)

# Tools
file(GLOB TOOLS_SRC "src/tools/*.cpp")
add_library(kacanopen_tools SHARED ${TOOLS_SRC})
target_link_libraries(${TARGET_NAME}
  ${CMAKE_DL_LIBS}
  ${CMAKE_THREAD_LIBS_INIT}
)

#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS kacanopen_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Install EDS library
message(STATUS "Configuring installation for EDS library")
add_definitions("-DSHARE_SOURCE_PATH=\"${CMAKE_CURRENT_LIST_DIR}/master/share\"")
add_definitions("-DSHARE_INSTALLED_PATH=\"${CMAKE_INSTALL_PREFIX}/share/kacanopen\"")
message(STATUS "SHARE_SOURCE_PATH set to ${CMAKE_CURRENT_LIST_DIR}/share.")
message(STATUS "SHARE_INSTALLED_PATH set to ${CMAKE_INSTALL_PREFIX}/share/kacanopen.")
install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/master/share/
	DESTINATION share/kacanopen
)

### Source code ###

if (NOT(${NO_ROS}))
	add_subdirectory(ros_bridge)
endif()

add_subdirectory(drivers_lgpl)
add_subdirectory(drivers)

