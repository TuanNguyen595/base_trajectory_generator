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

class BasePathGeneratorNode : public rclcpp::Node
{
public:
    using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<ComputePathToPose>;

    explicit BasePathGeneratorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    virtual ~BasePathGeneratorNode() = default;

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void generatorTimerCallback();
    void basePathCallback(const nav_msgs::msg::Path::SharedPtr msg);

    void goalResponseCallback(GoalHandle::SharedPtr goal);
    void feedbackCallback(GoalHandle::SharedPtr, const std::shared_ptr<const ComputePathToPose::Feedback> feedback);
    void resultCallback(const GoalHandle::WrappedResult & result);
    std::vector<std::pair<int, int>> bresenhamLine(int x0, int y0, int x1, int y1) ;
private:
    rclcpp_action::Client<ComputePathToPose>::SharedPtr cptp_client_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr base_trajectory_collision_pub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr base_path_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr safe_trajectory_pub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_pub_;
    nav_msgs::msg::OccupancyGrid grid_map_;
    rclcpp::TimerBase::SharedPtr generator_timer_;
    std::mutex mutex_;
    std::random_device rd_;
    std::mt19937 gen_;
    std::uniform_real_distribution<> dis_radians_;
    uint16_t number_waypoints_;
    double waypoint_dist_;
    std::string planner_id_;
    uint16_t generation_timer_interval_;

};
