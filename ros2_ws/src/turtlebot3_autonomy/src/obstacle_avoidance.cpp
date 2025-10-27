#include "turtlebot3_autonomy/obstacle_avoidance.hpp"

ObstacleAvoidance::ObstacleAvoidance() : Node("obstacle_avoidance")
{
  // Create subscription to laser scan data
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10, std::bind(&ObstacleAvoidance::laserCallback, this, std::placeholders::_1));
    
  // Create publisher for velocity commands
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
  RCLCPP_INFO(this->get_logger(), "Obstacle avoidance node initialized");
}

void ObstacleAvoidance::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  auto twist = geometry_msgs::msg::Twist();
    
  // Find minimum distance in front of the robot
  float min_distance = std::numeric_limits<float>::max();
  size_t center_index = msg->ranges.size() / 2;
  size_t range = msg->ranges.size() / 6;  // Check front 60 degrees
    
  for (size_t i = center_index - range; i < center_index + range; i++)
  {
    if (std::isfinite(msg->ranges[i]) && msg->ranges[i] < min_distance)
    {
      min_distance = msg->ranges[i];
    }
  }
    
  // Obstacle avoidance logic
  const float SAFE_DISTANCE = 0.5;  // meters
    
  if (min_distance < SAFE_DISTANCE)
  {
    // Obstacle detected - stop and turn
    twist.linear.x = 0.0;
    twist.angular.z = 0.5;
    RCLCPP_WARN(this->get_logger(), "Obstacle detected at %.2f m - turning", min_distance);
  }
  else
  {
    // No obstacle - move forward
    twist.linear.x = 0.2;
    twist.angular.z = 0.0;
  }
    
  cmd_vel_pub_->publish(twist);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleAvoidance>());
  rclcpp::shutdown();
  return 0;
}
