cmake_minimum_required(VERSION 3.0.2)
project(mine_uav_control)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  message_filters
  mavros_msgs
  nav_msgs
  pcl_conversions
  quadrotor_msgs
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  super_planner
  visualization_msgs
)

add_message_files(FILES ShaftDepthEstimate.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES arc_trajectory super_exploration_decider_lib
  CATKIN_DEPENDS geometry_msgs message_filters mavros_msgs nav_msgs
                 message_runtime
                 pcl_conversions quadrotor_msgs roscpp sensor_msgs std_msgs std_srvs
                 super_planner
                 visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(arc_trajectory
  src/arc_trajectory.cpp
)
target_link_libraries(arc_trajectory ${catkin_LIBRARIES})

add_executable(arc_trajectory_node
  src/arc_trajectory_node.cpp
)
target_link_libraries(arc_trajectory_node arc_trajectory ${catkin_LIBRARIES})

add_executable(offboard_bridge
  src/offboard_bridge.cpp
)
target_link_libraries(offboard_bridge ${catkin_LIBRARIES})

add_executable(fastlio_px4_vision_bridge
  src/fastlio_px4_vision_bridge.cpp
)
target_link_libraries(fastlio_px4_vision_bridge ${catkin_LIBRARIES})

add_executable(super_px4_command_bridge
  src/super_px4_command_bridge.cpp
)
target_link_libraries(super_px4_command_bridge ${catkin_LIBRARIES})

add_library(super_exploration_decider_lib
  src/super_exploration_decider.cpp
)
add_dependencies(super_exploration_decider_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(super_exploration_decider_lib ${catkin_LIBRARIES})

add_executable(super_exploration_decider
  src/super_exploration_decider_node.cpp
)
target_link_libraries(super_exploration_decider super_exploration_decider_lib
                      ${catkin_LIBRARIES})

add_executable(task1_perception_gate
  src/task1_perception_gate.cpp
)
target_link_libraries(task1_perception_gate ${catkin_LIBRARIES})

add_executable(mission_scheduler
  src/mission_scheduler.cpp
)
target_link_libraries(mission_scheduler ${catkin_LIBRARIES})

add_executable(shaft_mission_node
  src/shaft_mission_node.cpp
)
target_link_libraries(shaft_mission_node ${catkin_LIBRARIES})
add_dependencies(shaft_mission_node ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/super_bridge_manual_override.test)
  add_rostest(test/shaft_mission_ros.test)
  add_rostest(test/mission_scheduler_edges.test)
  add_rostest_gtest(test_super_goal_publication
    test/super_goal_publication.test
    test/test_super_goal_publication.cpp
  )
  target_link_libraries(test_super_goal_publication
    super_exploration_decider_lib ${catkin_LIBRARIES})
  add_dependencies(test_super_goal_publication ${catkin_EXPORTED_TARGETS})
  catkin_add_gtest(test_arc_trajectory
    test/test_arc_trajectory.cpp
  )
  if(TARGET test_arc_trajectory)
    target_link_libraries(test_arc_trajectory arc_trajectory ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_offboard_mode_guard
    test/test_offboard_mode_guard.cpp
  )
  catkin_add_gtest(test_anisotropic_clearance
    test/test_anisotropic_clearance.cpp
  )
  if(TARGET test_anisotropic_clearance)
    target_link_libraries(test_anisotropic_clearance ${GTEST_MAIN_LIBRARIES})
  endif()
  catkin_add_gtest(test_outbound_goal_contract
    test/test_outbound_goal_contract.cpp
  )
  if(TARGET test_outbound_goal_contract)
    target_link_libraries(test_outbound_goal_contract ${GTEST_MAIN_LIBRARIES})
  endif()
  catkin_add_gtest(test_shaft_mission test/test_shaft_mission.cpp)
  if(TARGET test_shaft_mission)
    target_link_libraries(test_shaft_mission ${GTEST_MAIN_LIBRARIES})
  endif()
endif()

install(TARGETS arc_trajectory arc_trajectory_node offboard_bridge
                fastlio_px4_vision_bridge
                super_px4_command_bridge
                super_exploration_decider_lib super_exploration_decider
                task1_perception_gate mission_scheduler shaft_mission_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/task1_real_time_guard.py
  scripts/analyze_task1_height_contract.py
  scripts/analyze_task1_sitl_bag.py
  scripts/ground_station.py
  scripts/gazebo_mid360_fastlio_adapter.py
  scripts/sitl_localization_adapter.py
  scripts/sitl_task_operator.py
  scripts/sitl_shaft_sensor_adapter.py
  scripts/sitl_shaft_px4_router.py
  scripts/sitl_shaft_release_pad.py
  scripts/sitl_shaft_takeover_injector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
)

install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)
