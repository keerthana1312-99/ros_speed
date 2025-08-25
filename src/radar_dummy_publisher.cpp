#include "rclcpp/rclcpp.hpp"
#include "speed_control/msg/radar_detections.hpp"
#include "speed_control/msg/radar_detection.hpp"

using namespace std::chrono_literals;

class RadarDummyPublisher : public rclcpp::Node
{
public:
  RadarDummyPublisher()
  : Node("radar_dummy_publisher")
  {
    publisher_ = this->create_publisher<speed_control::msg::RadarDetections>(
      "/radar_detections", 10);

    timer_ = this->create_wall_timer(
      1s, std::bind(&RadarDummyPublisher::publish_dummy_data, this));
  }

private:
  void publish_dummy_data()
  {
    auto msg = speed_control::msg::RadarDetections();

    // Example: one detection at 8.5m ahead, moving towards at -1.2 m/s
    speed_control::msg::RadarDetection det;
    det.distance = 8.5;
    det.relative_velocity = -1.2;
    msg.detections.push_back(det);

    publisher_->publish(msg);

    RCLCPP_INFO(this->get_logger(), "📡 Published dummy radar detection: distance=%.2f m, rel_vel=%.2f m/s",
                det.distance, det.relative_velocity);
  }

  rclcpp::Publisher<speed_control::msg::RadarDetections>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RadarDummyPublisher>());
  rclcpp::shutdown();
  return 0;
}

