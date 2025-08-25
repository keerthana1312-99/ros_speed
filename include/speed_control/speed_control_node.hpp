#ifndef SPEED_CONTROL__SPEED_CONTROL_NODE_HPP_
#define SPEED_CONTROL__SPEED_CONTROL_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "speed_control/msg/radar_detections.hpp"

class SpeedControlNode : public rclcpp::Node {
public:
  explicit SpeedControlNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // Callbacks
  void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void radarCallback(const speed_control::msg::RadarDetections::SharedPtr msg);
  void controlTimer();

  // Helpers
  float computeNearestLidarObstacle() const;  // meters (front sector within lane)
  std::pair<float, float> computeNearestRadarObstacle() const; // {dist, rel_vel}
  float computeTargetSpeed(float dist_lidar, float dist_radar, float rel_vel_radar);

  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;
  rclcpp::Subscription<speed_control::msg::RadarDetections>::SharedPtr sub_radar_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_target_speed_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Latest data
  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;
  speed_control::msg::RadarDetections::SharedPtr latest_radar_;

  // Parameters
  float max_speed_;
  float min_speed_;
  float stop_distance_;      // hard stop distance
  float time_headway_;       // for speed vs distance
  float lane_width_;
  float front_angle_deg_;
  float max_considered_range_;
  float lowpass_alpha_;      // speed smoothing 0..1 (1 = no smoothing)
  float emergency_brake_rel_closure_; // m/s threshold

  // State
  float smoothed_speed_;
};

#endif  // SPEED_CONTROL__SPEED_CONTROL_NODE_HPP_

