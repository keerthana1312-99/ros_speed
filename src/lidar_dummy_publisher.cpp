#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include <random>

class LidarDummyPublisher : public rclcpp::Node
{
public:
    LidarDummyPublisher() : Node("lidar_dummy_publisher")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar/points", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                         std::bind(&LidarDummyPublisher::publish_dummy_cloud, this));
        RCLCPP_INFO(this->get_logger(), "✅ LidarDummyPublisher started, publishing on /lidar/points");
    }

private:
    void publish_dummy_cloud()
    {
        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = this->get_clock()->now();
        cloud_msg.header.frame_id = "base_link";

        // Create PointCloud2 fields (x, y, z)
        cloud_msg.height = 1;
        cloud_msg.width = 50; // 50 random points
        cloud_msg.is_dense = false;
        cloud_msg.is_bigendian = false;

        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(cloud_msg.width);

        // Fill in random points
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

        std::default_random_engine gen;
        std::uniform_real_distribution<float> dist(-5.0, 5.0);

        for (size_t i = 0; i < cloud_msg.width; ++i, ++iter_x, ++iter_y, ++iter_z)
        {
            *iter_x = dist(gen);       // X coordinate
            *iter_y = dist(gen);       // Y coordinate
            *iter_z = dist(gen) / 2.0; // Z (smaller spread)
        }

        publisher_->publish(cloud_msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarDummyPublisher>());
    rclcpp::shutdown();
    return 0;
}

