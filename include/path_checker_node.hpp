/*
 * Author: Tuan Nguyen
 * Date: 11/24/2024
 */
#pragma once
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
#include "third_party/navfn.hpp"

#include <random>

class PathCheckerNode : public rclcpp::Node
{
public:
    using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<ComputePathToPose>;

    explicit PathCheckerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    virtual ~PathCheckerNode() = default;

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void basePathCallback(const nav_msgs::msg::Path::SharedPtr msg);

    std::vector<std::pair<int, int>> bresenhamLine(int x0, int y0, int x1, int y1) ;
private:
    std::vector<uint8_t> dilation(const std::vector<uint8_t>& data, int size_x, int size_y);
    bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my);
    void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy);
    bool getPlanFromPotential(const geometry_msgs::msg::Pose & goal, nav_msgs::msg::Path & plan);
    double getPointPotential(const geometry_msgs::msg::Point & world_point);
    void smoothApproachToGoal(const geometry_msgs::msg::Pose & goal, nav_msgs::msg::Path & plan);
    inline double squared_distance(
    const geometry_msgs::msg::Pose & p1,
    const geometry_msgs::msg::Pose & p2)
    {
        double dx = p1.position.x - p2.position.x;
        double dy = p1.position.y - p2.position.y;
        return dx * dx + dy * dy;
    }
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr base_trajectory_collision_pub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr base_path_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr safe_trajectory_pub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_pub_;
    nav_msgs::msg::OccupancyGrid grid_map_;
    std::string planner_id_;
    std::mutex mutex_;
    std::unique_ptr<NavFn> planner_;
    std::vector<uint8_t> map_data_;
};
