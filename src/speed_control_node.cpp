#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

// generated from your msg/ files
#include "speed_control/msg/radar_detection.hpp"
#include "speed_control/msg/radar_detections.hpp"

using speed_control::msg::RadarDetection;
using speed_control::msg::RadarDetections;

class SpeedControlNode : public rclcpp::Node
{
public:
    SpeedControlNode()
    : rclcpp::Node("speed_control_node")
    {
        // -------- Parameters (with defaults) --------
        max_speed_            = this->declare_parameter<float>("max_speed", 10.0f);
        min_speed_            = this->declare_parameter<float>("min_speed", 0.0f);
        stop_distance_        = this->declare_parameter<float>("stop_distance", 2.0f);
        time_headway_         = this->declare_parameter<float>("time_headway", 1.5f);
        lane_width_           = this->declare_parameter<float>("lane_width", 3.5f);
        front_angle_deg_      = this->declare_parameter<float>("front_angle_deg", 60.0f);
        max_considered_range_ = this->declare_parameter<float>("max_considered_range", 50.0f);
        lowpass_alpha_        = this->declare_parameter<float>("lowpass_alpha", 0.5f);
        emergency_brake_rel_closure_ =
            this->declare_parameter<float>("emergency_brake_rel_closure", -5.0f);
        radar_fov_deg_        = this->declare_parameter<float>("radar_fov_deg", 60.0f);

        smoothed_speed_ = 0.0f;

        auto qos = rclcpp::SensorDataQoS();

        // -------- Subscriptions --------
        sub_radar_ = this->create_subscription<RadarDetections>(
            "/radar_detections", qos,
            std::bind(&SpeedControlNode::radarCallback, this, std::placeholders::_1));

        sub_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/lidar/points", qos,
            std::bind(&SpeedControlNode::lidarCallback, this, std::placeholders::_1));

        // -------- Publishers --------
        pub_target_speed_ = this->create_publisher<std_msgs::msg::Float32>("/target_speed", 10);
        pub_cmd_vel_      = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        pub_markers_      = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/speed_control/markers", 10);

        // -------- Timer (20 Hz) --------
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&SpeedControlNode::controlTimer, this));

        RCLCPP_INFO(this->get_logger(), "SpeedControlNode started — waiting for radar/lidar data...");
    }

private:
    // --- Member Variables ---
    float max_speed_, min_speed_, stop_distance_, time_headway_;
    float lane_width_, front_angle_deg_, max_considered_range_;
    float lowpass_alpha_, emergency_brake_rel_closure_, radar_fov_deg_;
    float smoothed_speed_;

    sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;
    RadarDetections::SharedPtr latest_radar_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;
    rclcpp::Subscription<RadarDetections>::SharedPtr sub_radar_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_target_speed_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
    rclcpp::TimerBase::SharedPtr timer_;

    // --- Callbacks ---
    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    { latest_cloud_ = msg; }

    void radarCallback(const RadarDetections::SharedPtr msg)
    { latest_radar_ = msg; }

    float computeNearestLidarObstacle() const
    {
        if (!latest_cloud_) return std::numeric_limits<float>::infinity();

        const float half_lane = lane_width_ / 2.0f;
        const float front_angle_rad = front_angle_deg_ * static_cast<float>(M_PI) / 180.0f;

        float min_dist = std::numeric_limits<float>::infinity();

        sensor_msgs::PointCloud2ConstIterator<float> it_x(*latest_cloud_, "x");
        sensor_msgs::PointCloud2ConstIterator<float> it_y(*latest_cloud_, "y");

        for (; it_x != it_x.end(); ++it_x, ++it_y) {
            const float x = *it_x;
            const float y = *it_y;

            if (!std::isfinite(x) || !std::isfinite(y)) continue;
            if (x <= 0.0f) continue;
            if (x > max_considered_range_) continue;

            const float angle = std::atan2(y, x);
            if (std::fabs(angle) > front_angle_rad) continue;
            if (std::fabs(y) > half_lane) continue;

            min_dist = std::min(min_dist, x);
        }
        return min_dist;
    }

    std::pair<float,float> computeNearestRadarObstacle() const
    {
        if (!latest_radar_ || latest_radar_->detections.empty())
            return {std::numeric_limits<float>::infinity(), 0.0f};

        const float half_fov = radar_fov_deg_ * static_cast<float>(M_PI) / 180.0f;
        float min_dist = std::numeric_limits<float>::infinity();
        float rel_vel  = 0.0f;

        for (const auto& d : latest_radar_->detections) {
            if (!std::isfinite(d.distance) || !std::isfinite(d.angle) || !std::isfinite(d.relative_velocity))
                continue;
            if (d.distance <= 0.0f || d.distance > max_considered_range_) continue;
            if (std::fabs(d.angle) > half_fov) continue;

            if (d.distance < min_dist) {
                min_dist = d.distance;
                rel_vel  = d.relative_velocity;
            }
        }
        return {min_dist, rel_vel};
    }

    float computeTargetSpeed(float d_lidar, float d_radar, float rel_vel_radar)
    {
        const float d = std::min(d_lidar, d_radar);
        float v_desired;

        if (!std::isfinite(d) || d == std::numeric_limits<float>::infinity()) {
            v_desired = max_speed_;
        } else if (d <= stop_distance_) {
            v_desired = 0.0f;
        } else {
            v_desired = std::clamp((d - stop_distance_) / std::max(0.1f, time_headway_),
                                    min_speed_, max_speed_);
        }

        if (rel_vel_radar < emergency_brake_rel_closure_) {
            const float cap = std::max(0.0f, max_speed_ + rel_vel_radar);
            v_desired = std::min(v_desired, cap);
        }

        smoothed_speed_ = lowpass_alpha_ * v_desired + (1.0f - lowpass_alpha_) * smoothed_speed_;
        smoothed_speed_ = std::clamp(smoothed_speed_, min_speed_, max_speed_);
        return smoothed_speed_;
    }

    void controlTimer()
    {
        const float d_lidar = computeNearestLidarObstacle();
        const auto radar = computeNearestRadarObstacle();
        const float d_radar = radar.first;
        const float rel_vel = radar.second;

        const float target = computeTargetSpeed(d_lidar, d_radar, rel_vel);

        // Publish target speed
        std_msgs::msg::Float32 speed_msg;
        speed_msg.data = target;
        pub_target_speed_->publish(speed_msg);

        // Publish cmd_vel
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = target;
        pub_cmd_vel_->publish(cmd);

        // Markers
        visualization_msgs::msg::MarkerArray arr;

        visualization_msgs::msg::Marker text;
        text.header.frame_id = "base_link";
        text.header.stamp = now();
        text.ns = "speed_control";
        text.id = 1;
        text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::msg::Marker::ADD;
        text.pose.position.z = 1.2;
        text.scale.z = 0.4;
        text.color.a = 1.0;
        text.color.r = text.color.g = text.color.b = 1.0;
        text.text = "Target: " + std::to_string(static_cast<int>(target*3.6f)) + " km/h";
        arr.markers.push_back(text);

        if (std::isfinite(d_lidar)) {
            visualization_msgs::msg::Marker sp;
            sp.header.frame_id = "base_link";
            sp.header.stamp = now();
            sp.ns = "speed_control";
            sp.id = 2;
            sp.type = visualization_msgs::msg::Marker::SPHERE;
            sp.action = visualization_msgs::msg::Marker::ADD;
            sp.pose.position.x = d_lidar;
            sp.scale.x = sp.scale.y = sp.scale.z = 0.5;
            sp.color.a = 0.8; sp.color.r = 1.0; sp.color.g = 0.3; sp.color.b = 0.3;
            arr.markers.push_back(sp);
        }

        if (std::isfinite(d_radar)) {
            visualization_msgs::msg::Marker cb;
            cb.header.frame_id = "base_link";
            cb.header.stamp = now();
            cb.ns = "speed_control";
            cb.id = 3;
            cb.type = visualization_msgs::msg::Marker::CUBE;
            cb.action = visualization_msgs::msg::Marker::ADD;
            cb.pose.position.x = d_radar;
            cb.scale.x = cb.scale.y = cb.scale.z = 0.5;
            cb.color.a = 0.8; cb.color.r = 0.3; cb.color.g = 0.3; cb.color.b = 1.0;
            arr.markers.push_back(cb);
        }

        pub_markers_->publish(arr);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SpeedControlNode>());
    rclcpp::shutdown();
    return 0;
}

