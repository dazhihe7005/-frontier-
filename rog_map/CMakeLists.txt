cmake_minimum_required(VERSION 3.5)
project(rog_map)

set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3 -Wall -g -fPIC")
add_compile_options(-DQT_NO_VERSION_TAGGING)
add_compile_options(-Werror=unused-variable)
add_compile_options(-Werror=return-type)

# reduce warning
add_compile_options(-Wno-dev)
cmake_policy(SET CMP0057 NEW)
cmake_policy(SET CMP0054 NEW)
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE BOOL "Suppress developer warnings")

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# Define the voxelize and raycasting method
add_definitions(-DORIGIN_AT_CORNER)
string(TOUPPER $ENV{ROS_DISTRO} ROS_VERSION)
message(STATUS "ROS version: ${ROS_VERSION}")

message("Using ROS1")
# add a marco for ros1
add_definitions(-DUSE_ROS1)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        pcl_ros
        geometry_msgs
        nav_msgs
        message_generation
        visualization_msgs
        message_filters
)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES rog_map
        CATKIN_DEPENDS roscpp rospy std_msgs
        #  DEPENDS system_lib
)

include_directories(
        SYSTEM
        include
        ../../../devel/include
        ${catkin_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

set(THIRD_PARTY
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        -ldw
        yaml-cpp
)

file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp include/*.h include/*.hpp include/*.cpp)
add_library(rog_map STATIC ${srcs})

target_include_directories(rog_map PUBLIC include)
target_link_libraries(rog_map PUBLIC
        ${THIRD_PARTY})

if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(rog_map_robot_state_velocity_test
            test/robot_state_velocity_test.cpp)
    if(TARGET rog_map_robot_state_velocity_test)
        target_compile_definitions(rog_map_robot_state_velocity_test PRIVATE
                ROG_MAP_TEST_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/nearest_cell_alias.yaml")
        target_link_libraries(rog_map_robot_state_velocity_test
                rog_map
                ${THIRD_PARTY})
    endif()
    catkin_add_gtest(rog_map_nearest_cell_alias_test
            test/nearest_cell_alias_test.cpp)
    if(TARGET rog_map_nearest_cell_alias_test)
        target_compile_definitions(rog_map_nearest_cell_alias_test PRIVATE
                ROG_MAP_TEST_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/nearest_cell_alias.yaml")
        target_link_libraries(rog_map_nearest_cell_alias_test
                rog_map
                ${THIRD_PARTY})
    endif()
    catkin_add_gtest(rog_map_inflation_geometry_test
            test/inflation_geometry_test.cpp)
    if(TARGET rog_map_inflation_geometry_test)
        target_link_libraries(rog_map_inflation_geometry_test
                ${GTEST_MAIN_LIBRARIES})
    endif()
endif()
