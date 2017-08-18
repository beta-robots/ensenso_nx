cmake_minimum_required(VERSION 2.8.3)

# Cmake policy
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0005 NEW)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

# Project name and the type of project
PROJECT(ensenso_nx)
message(STATUS "Executig CMakeLists of project  ${PROJECT_NAME}.")

# DEBUG/RELEASE
IF (NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE "RELEASE")
ENDIF (NOT CMAKE_BUILD_TYPE)
message(STATUS "Configured to compile in ${CMAKE_BUILD_TYPE} mode.")

#Set Flags
SET(CMAKE_CXX_FLAGS_DEBUG "-g -Wall -D_REENTRANT")
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -D_REENTRANT")

#Set compiler according C++11 support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has C++11 support.")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has C++0x support.")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Find catkin macros and libraries
find_package(
    catkin REQUIRED
    COMPONENTS
        roscpp
        rospy
        dynamic_reconfigure
        std_msgs
        sensor_msgs
        pcl_ros
        pcl_conversions
)

# Find point cloud Library
FIND_PACKAGE(PCL 1.3 REQUIRED COMPONENTS)
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})
ADD_DEFINITIONS(${PCL_DEFINITIONS})


# Add module path where FindEnsenso.cmake is, and then Set Ensenso library and include paths
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "$ENV{ENSENSO_INSTALL}/cmake/")
find_package(Ensenso REQUIRED)
include_directories(${ENSENSO_INCLUDE_DIRS})
message(STATUS "Ensenso Headers found at ${ENSENSO_INCLUDE_DIRS}.")
message(STATUS "Ensenso Libraries found at ${ENSENSO_LIBRARIES}.")

# add other necessary include directories
include_directories(src/ ${catkin_INCLUDE_DIRS})

# Set source files
SET(SRCS_LIB
    src/ensenso_nx.cpp
)

SET(SRCS_NODE
    src/ensenso_nx_node.cpp
    src/ensenso_nx_node_main.cpp
)

#dynamic reconfigure cfg file
generate_dynamic_reconfigure_options(cfg/ensenso_nx_params.cfg)

## The catkin_package macro generates cmake config files for your package
catkin_package(
    CATKIN_DEPENDS
        roscpp
        rospy
        dynamic_reconfigure
        std_msgs
        sensor_msgs
        pcl_ros
        pcl_conversions
)

#Build library (TODO: Should call a CMakeLists.txt at src/lib/)
ADD_LIBRARY(${PROJECT_NAME}_LIB SHARED ${SRCS_LIB})
TARGET_LINK_LIBRARIES(${PROJECT_NAME}_LIB
                      ${ENSENSO_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${catkin_LIBRARIES})

#Build executable
ADD_EXECUTABLE(${PROJECT_NAME} ${SRCS_NODE} ${SRCS_LIB})
TARGET_LINK_LIBRARIES(${PROJECT_NAME}
                      ${ENSENSO_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

#Build examples
ADD_SUBDIRECTORY(src/examples)
