#include "mine_uav_control/super_exploration_decider.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "super_exploration_decider");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  mine_uav_control::SuperExplorationDecider decider(nh, private_nh);
  ros::spin();
  return 0;
}
