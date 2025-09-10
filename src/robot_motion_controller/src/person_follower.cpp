#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#define PI 3.14159265358

class PersonFollower : public rclcpp::Node
{
public:
    PersonFollower(): Node("person_follower")
    {
      // Declare parameters
      this->declare_parameter<float>("following_distance", 0.2);
      this->declare_parameter<float>("following_angle", 0);
      this->declare_parameter<float>("angle_control_gain", 1.0);
      this->declare_parameter<float>("distance_control_gain", 0.5);
      // Get parameter values
      this->get_parameter("following_distance", following_distance_);
      this->get_parameter("following_angle", following_angle_);
      this->get_parameter("angle_control_gain", angle_control_gain_);
      this->get_parameter("distance_control_gain", followindistance_control_gain_);
      // Print parameter values
      RCLCPP_INFO(this->get_logger(), "following_distance: %.2f", following_distance_);
      RCLCPP_INFO(this->get_logger(), "following_angle: %.2f", following_angle_);
      RCLCPP_INFO(this->get_logger(), "angle_control_gain: %.2f", angle_control_gain_);
      RCLCPP_INFO(this->get_logger(), "distance_control_gain: %.2f", followindistance_control_gain_);
      // Publisher for the topic /cmd_vel
      this->cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            rclcpp::SystemDefaultsQoS());
      using namespace std::placeholders;
      //Subsriber to the /scan topic
      this->scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
          "/scan",
          rclcpp::SensorDataQoS(),
          std::bind(&PersonFollower::scan_callback, this, _1)
      );
    }
private:
  // Define a command velocity publisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  // Define a laser scan subscriber
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
  // laser scan topic message pointer
  sensor_msgs::msg::LaserScan::SharedPtr scan_;
  // parameters
  float following_distance_;
  float following_angle_;
  float angle_control_gain_;
  float followindistance_control_gain_;
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
};

void PersonFollower::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
    auto min_distance = std::min_element(scan_msg->ranges.begin(), scan_msg->ranges.end());
    float min_value = *min_distance;
    int min_index = std::distance(scan_msg->ranges.begin(), min_distance);
    float min_angle = (min_index - 320)*2*PI/640.0;

    geometry_msgs::msg::Twist cmd_vel_msg;
    if(min_value < 12)
    {
        cmd_vel_msg.angular.z = angle_control_gain_*(min_angle - following_angle_);
        cmd_vel_msg.linear.x = followindistance_control_gain_*(min_value - following_distance_);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "No Object is Detected");
        cmd_vel_msg.linear.x = 0.0;
    }
    cmd_vel_publisher_->publish(cmd_vel_msg);
}

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<PersonFollower>());
	rclcpp::shutdown();
	return 0;
}