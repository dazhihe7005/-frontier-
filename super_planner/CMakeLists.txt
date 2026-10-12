cmake_minimum_required(VERSION 3.5)
project(super_planner)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3 -Wall -g -fPIC")
add_compile_options(-DQT_NO_VERSION_TAGGING)
add_compile_options(-Werror=return-type)
add_compile_options(-Werror=unused-variable)
add_compile_options(-Werror=unused-but-set-variable)
set(CMAKE_CXX_STANDARD 17)

# Define the voxelize and raycasting method
add_definitions(-DORIGIN_AT_CORNER)
add_definitions(-DFMT_HEADER_ONLY)
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")


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
        quadrotor_msgs
        visualization_msgs
        message_filters
        rog_map
        yaml-cpp
        tf
)

add_message_files(FILES GoalCommand.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem regex iostreams)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
catkin_package(
        INCLUDE_DIRS
        LIBRARIES super
        CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_runtime
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
        ${Boost_LIBRARIES}
        #        super_planner
        yaml-cpp
        -ldw
)

file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp include/*.h include/*.hpp)
add_library(super STATIC ${srcs})
target_include_directories(super PUBLIC
        include)
target_link_libraries(super PUBLIC
)
add_dependencies(super ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(fsm_node
        Apps/fsm_node_ros1.cpp
)
target_link_libraries(fsm_node
        super
        ${THIRD_PARTY}
)
add_dependencies(fsm_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(yaw_rate_limit_test test/yaw_rate_limit_test.cpp)
    if(TARGET yaw_rate_limit_test)
        target_link_libraries(yaw_rate_limit_test super ${THIRD_PARTY})
    endif()
    catkin_add_gtest(goal_command_validation_test test/goal_command_validation_test.cpp)
    if(TARGET goal_command_validation_test)
        target_link_libraries(goal_command_validation_test ${GTEST_MAIN_LIBRARIES})
        add_dependencies(goal_command_validation_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    endif()
    catkin_add_gtest(goal_command_message_test test/goal_command_message_test.cpp)
    if(TARGET goal_command_message_test)
        target_link_libraries(goal_command_message_test ${GTEST_MAIN_LIBRARIES})
        add_dependencies(goal_command_message_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    endif()
    catkin_add_gtest(goal_lifecycle_guard_test test/goal_lifecycle_guard_test.cpp)
    if(TARGET goal_lifecycle_guard_test)
        target_link_libraries(goal_lifecycle_guard_test ${GTEST_MAIN_LIBRARIES})
    endif()
    catkin_add_gtest(fsm_goal_cancellation_test test/fsm_goal_cancellation_test.cpp)
    if(TARGET fsm_goal_cancellation_test)
        target_link_libraries(fsm_goal_cancellation_test super ${THIRD_PARTY})
        add_dependencies(fsm_goal_cancellation_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    endif()
    catkin_add_gtest(super_guide_time_initialization_test
            test/guide_time_initialization_test.cpp)
    if(TARGET super_guide_time_initialization_test)
        target_link_libraries(super_guide_time_initialization_test
                super
                ${THIRD_PARTY})
    endif()
    catkin_add_gtest(line_search_plateau_test test/line_search_plateau_test.cpp)
    if(TARGET line_search_plateau_test)
        target_link_libraries(line_search_plateau_test ${GTEST_MAIN_LIBRARIES})
    endif()
    catkin_add_gtest(flight_z_contract_test test/flight_z_contract_test.cpp)
    if(TARGET flight_z_contract_test)
        target_link_libraries(flight_z_contract_test ${GTEST_MAIN_LIBRARIES})
    endif()
    catkin_add_gtest(ciri_tangent_plane_test test/ciri_tangent_plane_test.cpp)
    if(TARGET ciri_tangent_plane_test)
        target_link_libraries(ciri_tangent_plane_test super ${THIRD_PARTY} ${GTEST_MAIN_LIBRARIES})
        target_link_options(ciri_tangent_plane_test PRIVATE -fuse-ld=lld)
        add_dependencies(ciri_tangent_plane_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    endif()
    catkin_add_gtest(horizontal_clearance_contract_test test/horizontal_clearance_contract_test.cpp)
    if(TARGET horizontal_clearance_contract_test)
        target_link_libraries(horizontal_clearance_contract_test ${GTEST_MAIN_LIBRARIES})
    endif()
    catkin_add_gtest(path_start_guard_test test/path_start_guard_test.cpp)
    if(TARGET path_start_guard_test)
        target_link_libraries(path_start_guard_test ${GTEST_MAIN_LIBRARIES})
    endif()
endif()

#add_executable(read_replan_log
#        Apps/read_replan_log.cpp
#)
#target_link_libraries(read_replan_log
#        super
#        -lncurses
#        ${THIRD_PARTY}
#)
#
#
#add_executable(traj_opt_tuning
#        Apps/traj_opt_tuning.cpp
#)
#target_link_libraries(traj_opt_tuning
#        super
#        -lncurses
#        ${THIRD_PARTY}
#)
