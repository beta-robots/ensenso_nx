cmake_minimum_required(VERSION 2.8.3)

project(ensenso_nx)
message(STATUS "Executig CMakeLists of project  ${PROJECT_NAME}.")

# DEBUG/RELEASE
if (NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "RELEASE")
endif (NOT CMAKE_BUILD_TYPE)
message(STATUS "Configured to compile in ${CMAKE_BUILD_TYPE} mode.")

set(CMAKE_CXX_FLAGS_DEBUG "-g -Wall -D_REENTRANT")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -D_REENTRANT")

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

find_package(
    catkin REQUIRED
    COMPONENTS
	message_generation
        roscpp
        rospy
        dynamic_reconfigure
        std_msgs
        sensor_msgs
        pcl_ros
        pcl_conversions
	actionlib_msgs
)

find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(PCL 1.3 REQUIRED COMPONENTS)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "$ENV{ENSENSO_INSTALL}/cmake/")
find_package(Ensenso REQUIRED)
include_directories(${ENSENSO_INCLUDE_DIRS})
message(STATUS "Ensenso Headers found at ${ENSENSO_INCLUDE_DIRS}.")
message(STATUS "Ensenso Libraries found at ${ENSENSO_LIBRARIES}.")

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)



generate_dynamic_reconfigure_options(cfg/EnsensoNxParams.cfg)


add_action_files(
  DIRECTORY action
  FILES
  HECalibration.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs geometry_msgs
)


catkin_package(
	CATKIN_DEPENDS
	roscpp
	message_generation
        tf2
        tf2_ros
	rospy
	dynamic_reconfigure
	std_msgs
	sensor_msgs
	pcl_ros
	pcl_conversions
	actionlib_msgs
)




add_library(${PROJECT_NAME}_lib SHARED
	src/${PROJECT_NAME}/ensenso_nx.cpp
)
target_link_libraries(${PROJECT_NAME}_lib
	${ENSENSO_LIBRARIES}
	${PCL_COMMON_LIBRARIES}
	${PCL_IO_LIBRARIES}
	${OpenCV_LIBRARIES}
)

add_library(${PROJECT_NAME}_node SHARED
	src/${PROJECT_NAME}/ensenso_nx_node.cpp
)
add_dependencies(${PROJECT_NAME}_node
	${PROJECT_NAME}_lib
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	${PROJECT_NAME}_gencfg
	${catkin_EXPORTED_TARGETS}
	${PROJECT_NAME}_generate_messages_cpp
)
target_link_libraries(${PROJECT_NAME}_node
	${PROJECT_NAME}_lib
	${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}
	src/ensenso_nx_node_main.cpp
)
add_dependencies(${PROJECT_NAME}
	${PROJECT_NAME}_lib
        ${PROJECT_NAME}_generate_messages_cpp
)
target_link_libraries(${PROJECT_NAME}
	${PROJECT_NAME}_node
)
