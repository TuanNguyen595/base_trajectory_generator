/*
 * Author: Tuan Nguyen
 * Date: 11/24/2024
 */
#pragma once
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/bool.hpp"

#include <random>

class BasePathGeneratorNode : public rclcpp::Node
{
public:
    explicit BasePathGeneratorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    virtual ~BasePathGeneratorNode() = default;

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void generatorTimerCallback();
private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    nav_msgs::msg::OccupancyGrid grid_map_;
    rclcpp::TimerBase::SharedPtr generator_timer_;
    std::mutex mutex_;
    std::random_device rd_;
    std::mt19937 gen_;
    std::uniform_real_distribution<> dis_radians_;
    uint16_t number_waypoints_;
    double waypoint_dist_;
    uint16_t generation_timer_interval_;

};
