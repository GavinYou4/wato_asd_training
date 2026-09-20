#include <chrono>
#include <cmath>
#include <memory>

#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())), state_(State::WAITING_FOR_GOAL) {
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    current_map_ = *msg;
    have_map_ = true;

    if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL)
        planPath();
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    goal_ = *msg;
    goal_received_ = true;
    goal_start_time_ = this->now();
    state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
    planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    robot_pose_ = msg->pose.pose;
}

void PlannerNode::timerCallback()
{
    if (state_ != State::WAITING_FOR_ROBOT_TO_REACH_GOAL) return;

    if (goalReached()) {
        RCLCPP_INFO(this->get_logger(), "reached the goal");
        state_ = State::WAITING_FOR_GOAL;
        goal_received_ = false;
    }
    else if ((this->now() - goal_start_time_).seconds() > 30.0) {
        RCLCPP_WARN(this->get_logger(), "timed out trying to reach the goal, giving up");
        state_ = State::WAITING_FOR_GOAL;
        goal_received_ = false;
    }
    else {
        planPath();
    }
}

bool PlannerNode::goalReached()
{
    double dx = goal_.point.x - robot_pose_.position.x;
    double dy = goal_.point.y - robot_pose_.position.y;
    return std::sqrt(dx * dx + dy * dy) < 0.5;
}

void PlannerNode::planPath()
{
    if (!goal_received_ || !have_map_) return;

    nav_msgs::msg::Path path;
    path.header.stamp = this->now();
    path.header.frame_id = "sim_world";

    bool ok = planner_.planPath(current_map_,
        robot_pose_.position.x, robot_pose_.position.y,
        goal_.point.x, goal_.point.y, path);

    if (!ok) {
        RCLCPP_WARN(this->get_logger(), "couldnt find a path to the goal");
        return;
    }

    path_pub_->publish(path);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
