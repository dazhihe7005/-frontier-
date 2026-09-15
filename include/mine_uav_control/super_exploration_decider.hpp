#pragma once

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <visualization_msgs/MarkerArray.h>

namespace mine_uav_control {

struct VoxelKey {
  int x{0};
  int y{0};
  int z{0};

  bool operator==(const VoxelKey& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey& key) const {
    const std::size_t hx = std::hash<int>{}(key.x);
    const std::size_t hy = std::hash<int>{}(key.y);
    const std::size_t hz = std::hash<int>{}(key.z);
    return hx ^ (hy + static_cast<std::size_t>(0x9e3779b9) + (hx << 6) +
                 (hx >> 2)) ^
           (hz + static_cast<std::size_t>(0x9e3779b9) + (hy << 6) +
            (hy >> 2));
  }
};

class SuperExplorationDecider {
 public:
  SuperExplorationDecider(const ros::NodeHandle& nh,
                          const ros::NodeHandle& private_nh);

 private:
  using Cloud = sensor_msgs::PointCloud2;
  using Odom = nav_msgs::Odometry;
  using SyncPolicy =
      message_filters::sync_policies::ApproximateTime<Cloud, Odom>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;
  using VoxelSet = std::unordered_set<VoxelKey, VoxelKeyHash>;

  enum CellState : uint8_t { FREE = 1, OCCUPIED = 2 };

  enum class ExplorationPhase {
    kForwardPriority,
    kFrontierFallback,
  };

  struct FrontierCandidate {
    VoxelKey key;
    geometry_msgs::PoseStamped goal;
    double score{0.0};
    int unknown_neighbors{0};
  };

  struct ThreeWallCoverage {
    bool complete{false};
    bool end_wall_found{false};
    double end_depth{0.0};
    double vehicle_progress{0.0};
    double left_ratio{0.0};
    double right_ratio{0.0};
    double end_lateral_span{0.0};
    int left_max_gap_bins{0};
    int right_max_gap_bins{0};
    int expected_side_bins{0};
  };

  struct MapClosureStatus {
    bool complete{false};
    bool front_boundary_seen{false};
    double vehicle_progress{0.0};
    double no_frontier_duration{0.0};
    double stable_map_duration{0.0};
    std::size_t actionable_frontiers{0};
    std::size_t occupied_voxels{0};
  };

  void synchronizedCallback(const Cloud::ConstPtr& cloud,
                            const Odom::ConstPtr& odom);
  void decisionTimerCallback(const ros::TimerEvent& event);
  void missionEnableCallback(const std_msgs::Bool::ConstPtr& message);
  void returnRequestCallback(const std_msgs::Bool::ConstPtr& msg);
  void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg);
  bool enableCallback(std_srvs::SetBool::Request& request,
                      std_srvs::SetBool::Response& response);
  bool resetCallback(std_srvs::Trigger::Request& request,
                     std_srvs::Trigger::Response& response);

  bool loadParameters();
  bool dataIsFresh() const;
  void updateMap(const sensor_msgs::PointCloud2& cloud,
                 const geometry_msgs::PoseStamped& pose);
  void updateDirectionalEvidence(const sensor_msgs::PointCloud2& cloud,
                                 const geometry_msgs::PoseStamped& pose);
  ThreeWallCoverage evaluateThreeWallCoverage() const;
  MapClosureStatus evaluateMapClosure(
      const std::vector<FrontierCandidate>& candidates);
  void publishCoverageStatus(const ThreeWallCoverage& coverage,
                             const MapClosureStatus& closure);
  void pruneMap();
  void markFree(const VoxelKey& key);
  void markOccupied(const VoxelKey& key);
  VoxelKey positionToKey(double x, double y, double z) const;
  geometry_msgs::Point keyToPoint(const VoxelKey& key) const;
  bool isOccupied(const VoxelKey& key) const;
  bool isClearForVehicle(const VoxelKey& key) const;
  bool isKnownFree(const VoxelKey& key) const;
  VoxelSet reachableFreeVoxels() const;
  std::vector<FrontierCandidate> findFrontiers(VoxelSet& reachable) const;
  bool isNearCoveredGoal(const geometry_msgs::Point& point) const;
  bool selectAndPublishFrontier(
      const std::vector<FrontierCandidate>& candidates);
  bool publishForwardLookaheadGoal(const VoxelSet& reachable);
  bool publishEndApproachGoal(const ThreeWallCoverage& coverage);
  bool isForwardCandidate(const FrontierCandidate& candidate) const;
  double candidateHeadingAlignment(const FrontierCandidate& candidate) const;
  double candidateMissionProgress(const FrontierCandidate& candidate) const;
  double candidateMissionLateral(const FrontierCandidate& candidate) const;
  const FrontierCandidate* selectForwardCandidate(
      const std::vector<FrontierCandidate>& candidates) const;
  void updateExplorationPhase(
      const std::vector<FrontierCandidate>& candidates);
  const char* explorationPhaseName() const;
  void publishGoal(const geometry_msgs::PoseStamped& goal,
                   const std::string& reason);
  void beginReturnHome(const std::string& reason);
  void publishStatus(const std::string& state,
                     const std::string& detail = std::string());
  void publishVisualization(const std::vector<FrontierCandidate>& candidates);
  void clearMissionState();

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  message_filters::Subscriber<Cloud> cloud_subscriber_;
  message_filters::Subscriber<Odom> odom_subscriber_;
  std::unique_ptr<Synchronizer> synchronizer_;

  ros::Subscriber return_request_subscriber_;
  ros::Subscriber mission_enable_subscriber_;
  ros::Subscriber battery_subscriber_;
  ros::Publisher goal_publisher_;
  ros::Publisher status_publisher_;
  ros::Publisher finished_publisher_;
  ros::Publisher returning_publisher_;
  ros::Publisher model_complete_publisher_;
  ros::Publisher coverage_status_publisher_;
  ros::Publisher visualization_publisher_;
  ros::ServiceServer enable_service_;
  ros::ServiceServer reset_service_;
  ros::Timer decision_timer_;

  std::unordered_map<VoxelKey, uint8_t, VoxelKeyHash> voxels_;
  std::vector<geometry_msgs::Point> covered_goals_;

  geometry_msgs::PoseStamped current_pose_;
  geometry_msgs::PoseStamped home_pose_;
  geometry_msgs::PoseStamped current_goal_;
  ros::Time last_sync_time_;
  ros::Time first_data_time_;
  ros::Time last_frontier_time_;
  ros::Time goal_sent_time_;
  ros::Time last_actionable_frontier_time_;
  ros::Time last_significant_map_growth_time_;

  bool have_data_{false};
  bool have_home_{false};
  bool have_active_goal_{false};
  bool active_goal_is_end_approach_{false};
  bool exploration_started_{false};
  bool returning_home_{false};
  bool mission_finished_{false};
  bool enabled_{true};
  bool return_requested_{false};
  bool have_battery_{false};
  double battery_percentage_{-1.0};
  int reached_goal_count_{0};
  int three_wall_complete_streak_{0};
  int map_closure_complete_streak_{0};
  std::size_t map_growth_reference_count_{0};

  double mission_heading_yaw_{0.0};

  ExplorationPhase exploration_phase_{ExplorationPhase::kForwardPriority};
  bool left_wall_visible_{false};
  bool right_wall_visible_{false};
  bool front_obstacle_visible_{false};
  int side_wall_missing_streak_{0};
  int front_obstacle_streak_{0};

  std::string cloud_topic_;
  std::string odom_topic_;
  std::string goal_topic_;
  std::string world_frame_;
  std::string return_request_topic_;
  std::string mission_enable_topic_;
  std::string battery_topic_;
  std::string status_topic_;
  std::string finished_topic_;
  std::string returning_topic_;
  std::string model_complete_topic_;
  std::string coverage_status_topic_;
  std::string visualization_topic_;

  double voxel_resolution_{0.5};
  double max_map_radius_{40.0};
  double raycast_max_range_{40.0};
  double frontier_search_radius_{30.0};
  double max_exploration_radius_from_home_{35.0};
  double min_goal_distance_{2.0};
  double goal_reached_distance_{1.0};
  double goal_timeout_{30.0};
  double no_frontier_timeout_{15.0};
  double min_data_duration_{5.0};
  double candidate_spacing_{3.0};
  double vehicle_radius_{0.35};
  double min_observation_height_above_home_{0.5};
  double max_observation_height_above_home_{2.5};
  double data_timeout_{1.0};
  double decision_rate_{2.0};
  double sync_slop_{0.08};
  double battery_return_threshold_{0.20};
  double return_home_height_offset_{0.0};
  double distance_weight_{0.35};
  double information_weight_{1.0};
  double heading_priority_weight_{4.0};
  double fallback_heading_weight_{0.75};
  double forward_sector_deg_{70.0};
  double side_wall_sector_deg_{50.0};
  double side_wall_min_range_{1.0};
  double side_wall_max_range_{12.0};
  double front_obstacle_range_{6.0};
  double front_obstacle_sector_deg_{24.0};
  double front_obstacle_min_lateral_span_{1.0};
  double front_obstacle_min_vertical_span_{0.8};
  double forward_corridor_half_width_{2.0};
  double max_task_lateral_offset_{4.0};
  double forward_progress_weight_{2.0};
  double forward_lateral_penalty_{2.0};
  double forward_height_penalty_{1.0};
  double forward_goal_handover_distance_{3.0};
  double forward_lookahead_distance_{8.0};
  double forward_lookahead_step_{0.5};
  double cruise_height_above_home_{0.8};
  double directional_vertical_tolerance_{4.0};
  double directional_floor_exclusion_{0.5};
  double wall_coverage_bin_size_{1.0};
  double wall_coverage_min_depth_{8.0};
  double wall_coverage_end_min_depth_{12.0};
  double wall_coverage_min_ratio_{0.80};
  double wall_coverage_side_min_distance_{2.0};
  double wall_coverage_side_max_distance_{8.0};
  double wall_coverage_min_height_{0.3};
  double wall_coverage_max_height_{3.2};
  double wall_coverage_end_min_span_{4.0};
  double wall_coverage_end_center_half_width_{1.0};
  double wall_coverage_end_approach_distance_{6.0};
  double wall_coverage_end_standoff_distance_{3.0};
  int side_wall_missing_confirm_frames_{8};
  int front_obstacle_confirm_frames_{3};
  int front_obstacle_min_points_{20};
  int wall_coverage_max_gap_bins_{2};
  int wall_coverage_end_max_gap_bins_{2};
  int three_wall_confirm_cycles_{4};
  int sync_queue_size_{20};
  int max_points_per_cloud_{5000};
  int min_unknown_neighbors_{1};
  int min_goals_before_complete_{1};
  double map_closure_min_progress_{8.0};
  double map_closure_no_frontier_time_{4.0};
  double map_closure_stable_time_{4.0};
  int map_closure_max_actionable_frontiers_{0};
  int map_closure_growth_voxels_{500};
  int map_closure_confirm_cycles_{4};
  int max_reachable_voxels_{150000};
  int max_frontier_candidates_{500};
  bool raycast_enable_{true};
  bool strict_cloud_frame_{false};
  bool require_three_wall_completion_{true};
  bool use_map_closure_completion_{false};
  bool map_closure_require_front_boundary_{true};
};

}  // namespace mine_uav_control
