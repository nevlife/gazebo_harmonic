#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"

class VehicleSpeedPublisher : public rclcpp::Node
{
public:
  VehicleSpeedPublisher()
  : Node("vehicle_speed_publisher")
  {
    sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/wheel", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        std_msgs::msg::Float64 speed_msg;
        speed_msg.data = msg->twist.twist.linear.x;
        pub_->publish(speed_msg);
      });

    pub_ = create_publisher<std_msgs::msg::Float64>("/vehicle/speed", 10);
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VehicleSpeedPublisher>());
  rclcpp::shutdown();
  return 0;
}
