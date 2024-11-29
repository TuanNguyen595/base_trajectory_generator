#include "path_checker_node.hpp"

PathCheckerNode::PathCheckerNode(const rclcpp::NodeOptions & options) 
: Node("path_checker_node", options) {
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("obstacle_gridmap",
   rclcpp::SystemDefaultsQoS(),
   std::bind(&PathCheckerNode::mapCallback, this, std::placeholders::_1));
  base_path_sub_ = this->create_subscription<nav_msgs::msg::Path>("base_trajectory", rclcpp::SystemDefaultsQoS(),
   std::bind(&PathCheckerNode::basePathCallback, this, std::placeholders::_1));
  base_trajectory_collision_pub_ = this->create_publisher<std_msgs::msg::Bool>("base_trajectory_collision",
   rclcpp::SystemDefaultsQoS());
  planner_id_ = this->declare_parameter("planner_id", "SmacHybrid");
  robot_radius_ = this->declare_parameter("robot_radius", 0.3);
  safe_trajectory_pub_ = this->create_publisher<nav_msgs::msg::Path>("safe_trajectory", rclcpp::SystemDefaultsQoS());
  stop_pub_ = this->create_publisher<std_msgs::msg::Bool>("stop", rclcpp::SystemDefaultsQoS());
  planner_ = std::make_unique<NavFn>(0,0);
  inflate_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("inflate_map", rclcpp::SystemDefaultsQoS());
  collision_point_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("collision_point", rclcpp::SystemDefaultsQoS());
}

void PathCheckerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  grid_map_ = *msg;
  if (!planner_.get() ||
    planner_->nx != static_cast<int>(grid_map_.info.width) ||
    planner_->ny != static_cast<int>(grid_map_.info.height))
  {
    planner_->setNavArr(grid_map_.info.width, grid_map_.info.height);
  }
  map_data_.resize(grid_map_.info.width * grid_map_.info.height);
  for(int i = 0; i < map_data_.size(); i++) {
    if(grid_map_.data[i] == 100) {
      map_data_[i] = COST_OBS; //Obstacle
    } else {
      map_data_[i] = 0;
    }
  }
  map_data_ = dilation(map_data_, grid_map_.info.width, grid_map_.info.height, (int)(robot_radius_/grid_map_.info.resolution + 0.5));
  planner_->setCostmap(map_data_.data(), true, true);
  nav_msgs::msg::OccupancyGrid inflate_map;
  inflate_map.info = grid_map_.info;
  inflate_map.header = grid_map_.header;
  for(int i = 0; i < map_data_.size(); i++) {
    if(map_data_[i] == COST_OBS) {
      inflate_map.data.push_back(100);
    } else {
      inflate_map.data.push_back(0);
    }
  }
  inflate_map_pub_->publish(inflate_map);
}

void PathCheckerNode::basePathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  if(grid_map_.data.empty()) {
    RCLCPP_INFO(get_logger(), "Map is empty");
    return;
  }  
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
      if (map_data_[cells[j].first + cells[j].second * size_x] == COST_OBS) {
        is_collision_free = false;
        //Publish collision point
        visualization_msgs::msg::Marker collision_point;
        collision_point.header = grid_map_.header;
        collision_point.id = 1;
        collision_point.type = visualization_msgs::msg::Marker::SPHERE;
        collision_point.action = visualization_msgs::msg::Marker::ADD;
        collision_point.pose.position.x = cells[j].first * resolution + origin_x;
        collision_point.pose.position.y = cells[j].second * resolution + origin_y;
        collision_point.pose.position.z = 0.0;
        collision_point.pose.orientation.x = 0.0;
        collision_point.pose.orientation.y = 0.0;
        collision_point.pose.orientation.z = 0.0;
        collision_point.pose.orientation.w = 1.0;
        collision_point.scale.x = 0.3;
        collision_point.scale.y = 0.3;
        collision_point.scale.z = 0.3;
        collision_point.color.r = 1.0;
        collision_point.color.g = 0.0;
        collision_point.color.b = 0.0;
        collision_point.color.a = 1.0;
        collision_point_pub_->publish(collision_point);
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
  nav_msgs::msg::Path safe_path;
  if(!is_collision_free) {
    //generate safe path
    int map_start[2];
    int map_goal[2];
    map_start[0] = static_cast<unsigned int>((msg->poses.front().pose.position.x - origin_x) / resolution);
    map_start[1] = static_cast<unsigned int>((msg->poses.front().pose.position.y - origin_y) / resolution);
    map_goal[0] = static_cast<unsigned int>((msg->poses.back().pose.position.x - origin_x) / resolution);
    map_goal[1] = static_cast<unsigned int>((msg->poses.back().pose.position.y - origin_y) / resolution);
    planner_->setStart(map_goal);
    planner_->setGoal(map_start);
    planner_->calcNavFnAstar();
    double resolution = grid_map_.info.resolution;
    double tolerance = resolution * 2;
    geometry_msgs::msg::Pose p, best_pose;
    geometry_msgs::msg::Pose goal = msg->poses.back().pose;

    bool found_legal = false;

    p = goal;
    double potential = planner_->potarr[map_goal[1] * planner_->nx + map_goal[0]];
    if (potential < POT_HIGH) {
      RCLCPP_INFO(get_logger(), "Goal is reachable by itself %f", potential);
      // Goal is reachable by itself
      best_pose = p;
      found_legal = true;
    } else {
      RCLCPP_INFO(get_logger(), "Goal is not reachable, finding nearest reachable point with potential %f", potential);
      // Goal is not reachable. Trying to find nearest to the goal
      // reachable point within its tolerance region
      double best_sdist = std::numeric_limits<double>::max();

      p.position.y = goal.position.y - tolerance;
      while (p.position.y <= goal.position.y + tolerance) {
        p.position.x = goal.position.x - tolerance;
        while (p.position.x <= goal.position.x + tolerance) {
          potential = getPointPotential(p.position);
          double sdist = squared_distance(p, goal);
          if (potential < POT_HIGH && sdist < best_sdist) {
            best_sdist = sdist;
            best_pose = p;
            found_legal = true;
          }
          p.position.x += resolution;
        }
        p.position.y += resolution;
      }
    }
    if (found_legal) {
      RCLCPP_INFO(get_logger(), "Found legal point");
      getPlanFromPotential(best_pose, safe_path);
      smoothApproachToGoal(goal, safe_path);
    } else {
      std_msgs::msg::Bool msg_stop;
      msg_stop.data = true;
      stop_pub_->publish(msg_stop);
    }
  }
  safe_path.header = msg->header;
  safe_trajectory_pub_->publish(safe_path);
  // base_path_sub_.reset();
}

bool PathCheckerNode::getPlanFromPotential(const geometry_msgs::msg::Pose & goal, nav_msgs::msg::Path & plan) {
  // clear the plan, just in case
  plan.poses.clear();

  // Goal should be in global frame
  double wx = goal.position.x;
  double wy = goal.position.y;

  // the potential has already been computed, so we won't update our copy of the costmap
  unsigned int mx, my;
  if (!worldToMap(wx, wy, mx, my)) {
    RCLCPP_WARN(
      get_logger(),
      "The goal sent to the navfn planner is off the global costmap."
      " Planning will always fail to this goal.");
    return false;
  }

  int map_goal[2];
  map_goal[0] = mx;
  map_goal[1] = my;

  planner_->setStart(map_goal);

  const int & max_cycles = (grid_map_.info.width >= grid_map_.info.height) ?
    (grid_map_.info.width * 10) : (grid_map_.info.height * 10);

  int path_len = planner_->calcPath(max_cycles);
  if (path_len == 0) {
    return false;
  }

  auto cost = planner_->getLastPathCost();
  RCLCPP_INFO(
    get_logger(),
    "Path found, %d steps, %f cost\n", path_len, cost);

  // extract the plan
  float * x = planner_->getPathX();
  float * y = planner_->getPathY();
  int len = planner_->getPathLen();

  for (int i = len - 1; i >= 0; --i) {
    // convert the plan to world coordinates
    double world_x, world_y;
    mapToWorld(x[i], y[i], world_x, world_y);

    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = world_x;
    pose.pose.position.y = world_y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    plan.poses.push_back(pose);
  }

  return !plan.poses.empty();
}

std::vector<std::pair<int, int>> PathCheckerNode::bresenhamLine(int x0, int y0, int x1, int y1) {
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

std::vector<uint8_t> PathCheckerNode::dilation(const std::vector<uint8_t>& data, int size_x, int size_y, int extend) {
  int kRows = extend * 2 + 1;
  int kCols = kRows;
  int kCenterX = kRows / 2;
  int kCenterY = kCols / 2;

  // Output image (1D array)
  std::vector<uint8_t> output(data.size(), 0);

for (int y = 0; y < size_y; ++y) {
  for (int x = 0; x < size_x; ++x) {
      // Apply the kernel
      int outputIndex = y * size_x + x; // Convert 2D index to 1D index
      if(data[outputIndex] == COST_OBS) {
        for (int ky = 0; ky < kRows; ++ky) {
          for (int kx = 0; kx < kCols; ++kx) {
            int neighborY = y + (ky - kCenterY); // Neighbor row index
            int neighborX = x + (kx - kCenterX); // Neighbor column index

            // Check if the neighboring pixel is within bounds
            if (neighborY >= 0 && neighborY < size_y && neighborX >= 0 && neighborX < size_x) {
              int index = neighborY * size_x + neighborX; // Convert 2D index to 1D index
              output[index] = COST_OBS;
            }
          }
        }
      }

      // if(data[outputIndex] == 0 && maxVal == COST_OBS) {
      //   output[outputIndex] = maxVal - 2;  // Set the dilated pixel value
      // } else {
      // }
    }
  }

  return output;
}

bool PathCheckerNode::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) {
  double origin_x = grid_map_.info.origin.position.x;
  double origin_y = grid_map_.info.origin.position.y;
  double resolution = grid_map_.info.resolution;
  mx = static_cast<int>((wx - origin_x) / resolution);
  my = static_cast<int>((wy - origin_y) / resolution);
  return (mx >= 0 && mx < grid_map_.info.width && my >= 0 && my < grid_map_.info.height);
}

void PathCheckerNode::mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) {
  double origin_x = grid_map_.info.origin.position.x;
  double origin_y = grid_map_.info.origin.position.y;
  double resolution = grid_map_.info.resolution;
  wx = origin_x + mx * resolution;
  wy = origin_y + my * resolution;
}

double PathCheckerNode::getPointPotential(const geometry_msgs::msg::Point & world_point) {
  double wx = world_point.x;
  double wy = world_point.y;
  unsigned int mx, my;
  worldToMap(wx, wy, mx, my);
  unsigned int index = my * planner_->nx + mx;
  return planner_->potarr[index];
}

void PathCheckerNode::smoothApproachToGoal(const geometry_msgs::msg::Pose & goal, nav_msgs::msg::Path & plan) {
  // Replace the last pose of the computed path if it's actually further away
  // to the second to last pose than the goal pose.
  if (plan.poses.size() >= 2) {
    auto second_to_last_pose = plan.poses.end()[-2];
    auto last_pose = plan.poses.back();
    if (
      squared_distance(last_pose.pose, second_to_last_pose.pose) >
      squared_distance(goal, second_to_last_pose.pose))
    {
      plan.poses.back().pose = goal;
      return;
    }
  }
  geometry_msgs::msg::PoseStamped goal_copy;
  goal_copy.pose = goal;
  plan.poses.push_back(goal_copy);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathCheckerNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
