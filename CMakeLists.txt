cmake_minimum_required(VERSION 2.8.3)
project(eskf_odometry_ros)

## Find catkin macros and libraries
find_package(catkin REQUIRED)
# ******************************************************************** 
#                 Add catkin additional components here
# ******************************************************************** 
find_package(catkin REQUIRED COMPONENTS iri_base_algorithm geometry_msgs std_msgs nav_msgs roscpp roslib px_comm sensor_msgs tf)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# ******************************************************************** 
#           Add system and labrobotica dependencies here
# ******************************************************************** 
# find_package(eskf_odometry REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(atools REQUIRED)

# ******************************************************************** 
#           Add topic, service and action definition here
# ******************************************************************** 
## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

# ******************************************************************** 
#                 Add the dynamic reconfigure file 
# ******************************************************************** 
generate_dynamic_reconfigure_options(cfg/EskfOdom.cfg)

# ******************************************************************** 
#                 Add run time dependencies here
# ******************************************************************** 
catkin_package(
#  INCLUDE_DIRS 
#  LIBRARIES 
# ******************************************************************** 
#            Add ROS and IRI ROS run time dependencies
# ******************************************************************** 
 CATKIN_DEPENDS iri_base_algorithm geometry_msgs std_msgs nav_msgs roscpp roslib px_comm sensor_msgs tf
# ******************************************************************** 
#      Add system and labrobotica run time dependencies here
# ******************************************************************** 
 DEPENDS atools
#  DEPENDS eskf_odometry atools
)

###########
## Build ##
###########

# ******************************************************************** 
#                   Add the include directories 
# ******************************************************************** 
# include_directories(include ${catkin_INCLUDE_DIRS} ${eskf_odometry_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS}  ${atools_INCLUDE_DIR})
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS}  ${atools_INCLUDE_DIR})


# MESSAGE(${eskf_odometry_INCLUDE_DIR})

## Declare a cpp library
add_library(eskf_odometry src/eskf_odometry.cpp)

## Declare a cpp executable
add_executable(${PROJECT_NAME} src/eskf_odometry_ros_alg.cpp src/eskf_odometry_ros_alg_node.cpp)
# add_executable(${PROJECT_NAME} src/eskf_odom.cpp)

# ******************************************************************** 
#                   Add the libraries
# ******************************************************************** 
# target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${eskf_odometry_LIBRARY} ${atools_LIBRARY})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${atools_LIBRARY} eskf_odometry)

# ******************************************************************** 
#               Add message headers dependencies 
# ******************************************************************** 
# add_dependencies(${PROJECT_NAME} <msg_package_name>_generate_messages_cpp)
add_dependencies(${PROJECT_NAME} geometry_msgs_generate_messages_cpp)
add_dependencies(${PROJECT_NAME} std_msgs_generate_messages_cpp)
add_dependencies(${PROJECT_NAME} nav_msgs_generate_messages_cpp)
add_dependencies(${PROJECT_NAME} roscpp_generate_messages_cpp)
add_dependencies(${PROJECT_NAME} px_comm_generate_messages_cpp)
add_dependencies(${PROJECT_NAME} sensor_msgs_generate_messages_cpp)
# ******************************************************************** 
#               Add dynamic reconfigure dependencies 
# ******************************************************************** 
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})

# Check G++ compiler version
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(CMAKE_BUILD_TYPE Release) 
