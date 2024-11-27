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

    void goalResponseCallback(GoalHandle::SharedPtr goal);
    void feedbackCallback(GoalHandle::SharedPtr, const std::shared_ptr<const ComputePathToPose::Feedback> feedback);
    void resultCallback(const GoalHandle::WrappedResult & result);
    std::vector<std::pair<int, int>> bresenhamLine(int x0, int y0, int x1, int y1) ;
private:
    rclcpp_action::Client<ComputePathToPose>::SharedPtr cptp_client_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr base_trajectory_collision_pub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr base_path_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr safe_trajectory_pub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_pub_;
    nav_msgs::msg::OccupancyGrid grid_map_;
    std::string planner_id_;
    std::mutex mutex_;

};
