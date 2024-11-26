#include "base_path_generator_node.hpp"

BasePathGeneratorNode::BasePathGeneratorNode(const rclcpp::NodeOptions & options) 
: Node("base_path_generator_node", options),
  gen_(rd_()), 
  dis_radians_(-M_PI, M_PI) {
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("obstacle_gridmap",
   rclcpp::SystemDefaultsQoS(),
   std::bind(&BasePathGeneratorNode::mapCallback, this, std::placeholders::_1));
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("base_trajectory", rclcpp::SystemDefaultsQoS());
  base_path_sub_ = this->create_subscription<nav_msgs::msg::Path>("base_trajectory", rclcpp::SystemDefaultsQoS(),
   std::bind(&BasePathGeneratorNode::basePathCallback, this, std::placeholders::_1));
  base_trajectory_collision_pub_ = this->create_publisher<std_msgs::msg::Bool>("base_trajectory_collision",
   rclcpp::SystemDefaultsQoS());
  number_waypoints_ = this->declare_parameter("number_waypoints", 10);
  waypoint_dist_ = this->declare_parameter("waypoint_dist", 1.0);
  planner_id_ = this->declare_parameter("planner_id", "SmacHybrid");
  generation_timer_interval_ = this->declare_parameter("generation_timer_interval", 2000.0);
  cptp_client_ = rclcpp_action::create_client<ComputePathToPose>(this, "compute_path_to_pose");
  safe_trajectory_pub_ = this->create_publisher<nav_msgs::msg::Path>("safe_trajectory", rclcpp::SystemDefaultsQoS());
  stop_pub_ = this->create_publisher<std_msgs::msg::Bool>("stop", rclcpp::SystemDefaultsQoS());
  generator_timer_ = this->create_wall_timer(std::chrono::milliseconds(generation_timer_interval_),
   std::bind(&BasePathGeneratorNode::generatorTimerCallback, this));
}

void BasePathGeneratorNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  grid_map_ = *msg;
}

void BasePathGeneratorNode::basePathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  mutex_.lock();
  double resolution = grid_map_.info.resolution;
  double origin_x = grid_map_.info.origin.position.x;
  double origin_y = grid_map_.info.origin.position.y;
  double size_x = grid_map_.info.width;
  double size_y = grid_map_.info.height;
  std::vector<int8_t> data = grid_map_.data;
  mutex_.unlock();
  
  bool is_collision_free = true;
  for (int i = 0; i < msg->poses.size() - 1; i++) {
    uint16_t smx, smy, emx, emy;
    smx = static_cast<unsigned int>((msg->poses[i].pose.position.x - origin_x) / resolution);
    smy = static_cast<unsigned int>((msg->poses[i].pose.position.y - origin_y) / resolution);
    emx = static_cast<unsigned int>((msg->poses[i + 1].pose.position.x - origin_x) / resolution);
    emy = static_cast<unsigned int>((msg->poses[i + 1].pose.position.y - origin_y) / resolution);
    if(emx >= size_x || emy >= size_y) {
      is_collision_free = false;
      RCLCPP_INFO(get_logger(), "Path is out of boundary");
      break;
    }
    std::vector<std::pair<int, int>> cells = bresenhamLine(smx, smy, emx, emy);
    for (unsigned int j = 0; j < cells.size(); j++) {
      if (data[cells[j].first + cells[j].second * size_x] == 100) {
        is_collision_free = false;
        break;
      }
    }
    if (!is_collision_free) {
      break;
    }
  }
  if (is_collision_free) {
    RCLCPP_INFO(get_logger(), "Path is collision free");
  } else {
    RCLCPP_INFO(get_logger(), "Path is not collision free, generate safe trajectory");
  }
  std_msgs::msg::Bool msg_collision_free;
  msg_collision_free.data = is_collision_free;
  base_trajectory_collision_pub_->publish(msg_collision_free);

  //Send goal
  if(!is_collision_free) {
    auto goal = ComputePathToPose::Goal();
    goal.goal = msg->poses.back();
    goal.planner_id = planner_id_;
    auto send_goal_options = rclcpp_action::Client<ComputePathToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&BasePathGeneratorNode::goalResponseCallback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&BasePathGeneratorNode::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(&BasePathGeneratorNode::resultCallback, this, std::placeholders::_1);
    cptp_client_->async_send_goal(goal, send_goal_options);
  } else {
    nav_msgs::msg::Path safe_path;
    safe_path.header = msg->header;
    safe_trajectory_pub_->publish(safe_path);
  }

  // base_path_sub_.reset();
}
void BasePathGeneratorNode::goalResponseCallback(GoalHandle::SharedPtr goal) {
  if(!goal) {
    RCLCPP_ERROR(get_logger(), "ComputePathToPose action client failed to send goal to server.");
  } else {
    RCLCPP_INFO(get_logger(), "ComputePathToPose action client sent goal to server.");
  }
}

void BasePathGeneratorNode::feedbackCallback(GoalHandle::SharedPtr, const std::shared_ptr<const ComputePathToPose::Feedback> feedback) {
  (void)feedback;
}

void BasePathGeneratorNode::resultCallback(const GoalHandle::WrappedResult & result) {
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
    {
      nav_msgs::msg::Path safe_path;
      safe_path = result.result->path;
      safe_trajectory_pub_->publish(safe_path);
      break;
    }
    case rclcpp_action::ResultCode::ABORTED:
    {
      std_msgs::msg::Bool stop_msg;
      stop_msg.data = true;
      stop_pub_->publish(stop_msg);
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      break;
    }
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code %d", (int)result.code);
      break;
  }
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

std::vector<std::pair<int, int>> BasePathGeneratorNode::bresenhamLine(int x0, int y0, int x1, int y1) {
  std::vector<std::pair<int, int>> cells;

  int dx = abs(x1 - x0);
  int dy = abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;

  while (true) {
    cells.emplace_back(x0, y0);

    if (x0 == x1 && y0 == y1) break;

    int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x0 += sx;
    }
    if (e2 < dx) {
      err += dx;
        y0 += sy  ;
    }
  }
  return cells;
}



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BasePathGeneratorNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
