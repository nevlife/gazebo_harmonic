#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>

class GpsCovarianceRelay : public rclcpp::Node {
public:
  GpsCovarianceRelay() : Node("gps_covariance_relay") {
    // xacro stddev 0.00000045 deg ≈ 0.05 m -> variance 0.0025 m²
    this->declare_parameter("horizontal_variance", 0.0025);
    this->declare_parameter("vertical_variance", 0.0025);
    h_var_ = this->get_parameter("horizontal_variance").as_double();
    v_var_ = this->get_parameter("vertical_variance").as_double();

    rclcpp::QoS qos(1);
    qos.best_effort();

    sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("gps/raw", qos, [this](sensor_msgs::msg::NavSatFix::SharedPtr msg) {
      msg->position_covariance = {
        h_var_, 0.0, 0.0,
        0.0, h_var_, 0.0,
        0.0, 0.0, v_var_
      };
      msg->position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
      msg->status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
      msg->status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
      pub_->publish(*msg);
    });
    pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps/fix", 10);
  }

private:
  double h_var_;
  double v_var_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GpsCovarianceRelay>());
  rclcpp::shutdown();
  return 0;
}
