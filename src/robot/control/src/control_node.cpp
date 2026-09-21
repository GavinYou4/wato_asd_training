#include <chrono>
#include <cmath>
#include <memory>

#include "control_node.hpp"

ControlNode::ControlNode()
  : Node("control"),
    control_(robot::ControlCore(this->get_logger())),
    lookahead_distance_(1.5),
    goal_tolerance_(0.5),
    linear_speed_(1.5),
    goal_reached_(false) {
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) {
      if (goal_reached_ && robot_odom_ && !msg->poses.empty() &&
          computeDistance(robot_odom_->pose.pose.position, msg->poses.back().pose.position) > goal_tolerance_) {
        goal_reached_ = false;
      }
      current_path_ = msg;
    });

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      robot_odom_ = msg;
    });

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  arrival_pub_ = this->create_publisher<std_msgs::msg::String>("/arrival", 10);

  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), [this]() { controlLoop(); });
}

void ControlNode::controlLoop() {
  if (!current_path_ || !robot_odom_ || current_path_->poses.empty()) {
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
    return;
  }

  const auto &robot_position = robot_odom_->pose.pose.position;
  const auto &goal_position = current_path_->poses.back().pose.position;

  if (goal_reached_ || computeDistance(robot_position, goal_position) < goal_tolerance_) {
    if (!goal_reached_) {
      goal_reached_ = true;
      std_msgs::msg::String arrival_msg;
      arrival_msg.data = "robot has arrived!";
      arrival_pub_->publish(arrival_msg);
      RCLCPP_INFO(this->get_logger(), "robot has arrived!");
    }
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
    return;
  }

  auto lookahead_point = findLookaheadPoint();
  if (!lookahead_point) {
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
    return;
  }

  cmd_vel_pub_->publish(computeVelocity(*lookahead_point));
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() {
  const auto &robot_position = robot_odom_->pose.pose.position;

  for (const auto &pose : current_path_->poses) {
    if (computeDistance(robot_position, pose.pose.position) >= lookahead_distance_) {
      return pose;
    }
  }

  return current_path_->poses.back();
}

geometry_msgs::msg::Twist ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped &target) {
  const auto &robot_position = robot_odom_->pose.pose.position;
  double yaw = extractYaw(robot_odom_->pose.pose.orientation);

  double dx = target.pose.position.x - robot_position.x;
  double dy = target.pose.position.y - robot_position.y;
  double alpha = std::atan2(dy, dx) - yaw;
  alpha = std::atan2(std::sin(alpha), std::cos(alpha));

  geometry_msgs::msg::Twist cmd_vel;
  if (std::fabs(alpha) > M_PI / 2.0) {
    cmd_vel.angular.z = alpha > 0.0 ? 1.5 : -1.5;
    return cmd_vel;
  }
  cmd_vel.linear.x = linear_speed_;
  cmd_vel.angular.z = 2.0 * linear_speed_ * std::sin(alpha) / lookahead_distance_;
  return cmd_vel;
}

double ControlNode::computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
  return std::hypot(b.x - a.x, b.y - a.y);
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion &q) {
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
