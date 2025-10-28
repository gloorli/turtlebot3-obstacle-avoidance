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
    
  // Find minimum distance in front of the robot and determine turn direction
  float min_distance = std::numeric_limits<float>::max();
  int turn_direction = 0;   // 1: left, -1: right
  size_t center_index = 0;  // index for 0 degree
  size_t ranges_size = msg->ranges.size();
  size_t check_range = ranges_size / 6;  // Check front 60 degrees
  
  for (size_t i = center_index; i < center_index + check_range/2; i++)
  {
    if (std::isfinite(msg->ranges[i]) && msg->ranges[i] < min_distance)
    {
      min_distance = msg->ranges[i];
      turn_direction = -1;
    }
  }
  for (size_t i = ranges_size - check_range/2; i < ranges_size; i++)
  {
    if (std::isfinite(msg->ranges[i]) && msg->ranges[i] < min_distance)
    {
      min_distance = msg->ranges[i];
      turn_direction = 1;
    }
  }

  // Obstacle avoidance logic
  const float SAFE_DISTANCE = 0.3;  // meters
    
  if (min_distance < SAFE_DISTANCE)
  {
    // Obstacle detected - slow down and turn
    twist.linear.x = 0.01;
    twist.angular.z = turn_direction * 0.5;
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
