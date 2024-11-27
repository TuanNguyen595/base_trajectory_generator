#include "base_path_generator_node.hpp"

BasePathGeneratorNode::BasePathGeneratorNode(const rclcpp::NodeOptions & options) 
: Node("base_path_generator_node", options),
  gen_(rd_()), 
  dis_radians_(-M_PI, M_PI) {
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("obstacle_gridmap",
   rclcpp::SystemDefaultsQoS(),
   std::bind(&BasePathGeneratorNode::mapCallback, this, std::placeholders::_1));
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("base_trajectory", rclcpp::SystemDefaultsQoS());
  number_waypoints_ = this->declare_parameter("number_waypoints", 10);
  waypoint_dist_ = this->declare_parameter("waypoint_dist", 1.0);
  generation_timer_interval_ = this->declare_parameter("generation_timer_interval", 2000.0);
  generator_timer_ = this->create_wall_timer(std::chrono::milliseconds(generation_timer_interval_),
   std::bind(&BasePathGeneratorNode::generatorTimerCallback, this));
}

void BasePathGeneratorNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  grid_map_ = *msg;
}

void BasePathGeneratorNode::generatorTimerCallback() {
  if(grid_map_.data.empty()) {
    RCLCPP_INFO(get_logger(), "Map is empty");
    return;
  }
  double angle = dis_radians_(gen_);
  nav_msgs::msg::Path base_path;
  base_path.header = grid_map_.header;
  base_path.poses.resize(number_waypoints_);
  for (int i = 0; i < number_waypoints_; i++) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = grid_map_.header;
    pose.pose.position.x = i * waypoint_dist_ * cos(angle);
    pose.pose.position.y = i * waypoint_dist_ * sin(angle);
    base_path.poses[i] = pose;
  }
  path_pub_->publish(base_path);
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BasePathGeneratorNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
