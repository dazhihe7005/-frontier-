#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <ros/ros.h>

#include <cmath>
#include <string>
#include <vector>

class PointCloud1ToPointCloud2 {
 public:
  PointCloud1ToPointCloud2() : private_nh_("~") {
    private_nh_.param("input_topic", input_topic_,
                      std::string("/mine_uav/sitl/mid360/points"));
    private_nh_.param("output_topic", output_topic_,
                      std::string("/mine_uav/sitl/mid360/points2"));
    private_nh_.param("min_range", min_range_, 0.20);
    private_nh_.param("max_range", max_range_, 30.0);
    private_nh_.param("max_range_epsilon", max_range_epsilon_, 0.05);
    private_nh_.param("self_filter_enable", self_filter_enable_, true);
    private_nh_.param("sensor_offset_x", sensor_offset_x_, 0.1315);
    private_nh_.param("sensor_offset_y", sensor_offset_y_, 0.0);
    private_nh_.param("sensor_offset_z", sensor_offset_z_, 0.223);
    private_nh_.param("sensor_pitch", sensor_pitch_, 0.436332313);
    private_nh_.param("self_filter_xy_radius", self_filter_xy_radius_, 0.62);
    private_nh_.param("self_filter_z_min", self_filter_z_min_, -0.32);
    private_nh_.param("self_filter_z_max", self_filter_z_max_, 0.35);
    publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 2);
    subscriber_ = nh_.subscribe(input_topic_, 2,
                                &PointCloud1ToPointCloud2::callback, this);
    ROS_INFO("MID360 PointCloud converter: %s -> %s; self filter %s, "
             "body cylinder r=%.3f z=[%.3f, %.3f] m",
             input_topic_.c_str(), output_topic_.c_str(),
             self_filter_enable_ ? "enabled" : "disabled",
             self_filter_xy_radius_, self_filter_z_min_, self_filter_z_max_);
  }

 private:
  bool isSelfReturn(const geometry_msgs::Point32& point) const {
    if (!self_filter_enable_) {
      return false;
    }

    // The Gazebo points are in the pitched MID360 frame. Transform each point
    // back into the FLU vehicle base frame before applying the physical
    // airframe envelope. The +25 degree SDF pitch points the sensor +X axis
    // toward vehicle -Z.
    const double cosine = std::cos(sensor_pitch_);
    const double sine = std::sin(sensor_pitch_);
    const double body_x = sensor_offset_x_ + cosine * point.x + sine * point.z;
    const double body_y = sensor_offset_y_ + point.y;
    const double body_z = sensor_offset_z_ - sine * point.x + cosine * point.z;
    const double radius_squared = body_x * body_x + body_y * body_y;
    return radius_squared <= self_filter_xy_radius_ * self_filter_xy_radius_ &&
           body_z >= self_filter_z_min_ && body_z <= self_filter_z_max_;
  }

  void callback(const sensor_msgs::PointCloud::ConstPtr& input) {
    std::vector<const geometry_msgs::Point32*> valid_points;
    valid_points.reserve(input->points.size());
    const double maximum_valid_range = max_range_ - max_range_epsilon_;
    std::size_t self_returns = 0;
    std::size_t censored_returns = 0;
    for (const auto& point : input->points) {
      const double range = std::sqrt(
          static_cast<double>(point.x) * point.x +
          static_cast<double>(point.y) * point.y +
          static_cast<double>(point.z) * point.z);
      // gazebo_ros_block_laser encodes a missed ray as a finite point exactly
      // at max_range.  Feeding those points to FAST-LIO2 creates a moving
      // spherical shell and destroys translational scan matching.
      if (!std::isfinite(range) || range < min_range_ ||
          range >= maximum_valid_range) {
        ++censored_returns;
        continue;
      }
      if (isSelfReturn(point)) {
        ++self_returns;
        continue;
      }
      valid_points.push_back(&point);
    }

    sensor_msgs::PointCloud2 output;
    output.header = input->header;
    output.height = 1;
    output.width = valid_points.size();
    output.is_bigendian = false;
    output.is_dense = false;

    sensor_msgs::PointCloud2Modifier modifier(output);
    modifier.setPointCloud2Fields(
        4, "x", 1, sensor_msgs::PointField::FLOAT32,
        "y", 1, sensor_msgs::PointField::FLOAT32,
        "z", 1, sensor_msgs::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::PointField::FLOAT32);
    modifier.resize(valid_points.size());

    sensor_msgs::PointCloud2Iterator<float> x(output, "x");
    sensor_msgs::PointCloud2Iterator<float> y(output, "y");
    sensor_msgs::PointCloud2Iterator<float> z(output, "z");
    sensor_msgs::PointCloud2Iterator<float> intensity(output, "intensity");
    for (const auto* point : valid_points) {
      *x = point->x;
      *y = point->y;
      *z = point->z;
      *intensity = 0.0F;
      ++x;
      ++y;
      ++z;
      ++intensity;
    }
    ROS_INFO_STREAM_THROTTLE(
        10.0, "MID360 preprocessing kept " << valid_points.size() << " / "
                                             << input->points.size()
                                             << " returns; self="
                                             << self_returns << ", range="
                                             << censored_returns);
    publisher_.publish(output);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber subscriber_;
  ros::Publisher publisher_;
  std::string input_topic_;
  std::string output_topic_;
  double min_range_;
  double max_range_;
  double max_range_epsilon_;
  double sensor_offset_x_;
  double sensor_offset_y_;
  double sensor_offset_z_;
  double sensor_pitch_;
  double self_filter_xy_radius_;
  double self_filter_z_min_;
  double self_filter_z_max_;
  bool self_filter_enable_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "pointcloud1_to_pointcloud2");
  PointCloud1ToPointCloud2 converter;
  ros::spin();
  return 0;
}
