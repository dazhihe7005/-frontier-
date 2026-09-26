cmake_minimum_required(VERSION 3.0.2)
project(mine_uav_gbplanner)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs mavros_msgs nav_msgs roscpp sensor_msgs std_msgs std_srvs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(pointcloud1_to_pointcloud2 src/pointcloud1_to_pointcloud2.cpp)
target_link_libraries(pointcloud1_to_pointcloud2 ${catkin_LIBRARIES})

add_executable(fastlio_px4_vision_bridge src/fastlio_px4_vision_bridge.cpp)
target_link_libraries(fastlio_px4_vision_bridge ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/fastlio_voxblox_adapter.py
  scripts/fastlio_planner_odometry.py
  scripts/gbplanner_decision_monitor.py
  scripts/diagnostic_map_coverage.py
  scripts/diagnostic_truth_trajectory.py
  scripts/gbplanner_mission_trigger.py
  scripts/gbplanner_px4_executor.py
  scripts/gbplanner_stall_recovery.py
  scripts/sitl_task_operator.py
  scripts/validate_full_chain.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS pointcloud1_to_pointcloud2 fastlio_px4_vision_bridge
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch rviz docs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
