#include <chrono>
#include <cmath>
#include <memory>

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
  timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));
  
  //40m x 40m global map 0.1m per cell centered around origin
  map_memory_.initGlobalMap(400, 400, 0.1, -20.0, -20.0);
}

//stores latest costmap message, tells the node that a new costmap has been received
void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    latest_costmap_ = *msg;
    costmap_updated_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    //updates robots position and orientation
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;

    auto q = msg->pose.pose.orientation;
    robot_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    have_odom_ = true;

    //calculating travel distance since last update
    double dx = robot_x_ - last_x_;
    double dy = robot_y_ - last_y_;
    double dist = std::sqrt(dx*dx + dy*dy);

    //only update map if robot's travel distance is more than the threshold
    if (dist >= distance_threshold_) {
        last_x_ = robot_x_;
        last_y_ = robot_y_;
        should_update_map_ = true;
    }
}

//every second, this function runs
void MapMemoryNode::updateMap()
{
    //make sure there is odometry data
    if (!have_odom_) return;

    //make sure there's a new costmap and either its the first update or the robot has travelled far enough for an update
    if (costmap_updated_ && (should_update_map_ || !first_update_done_)) {
        if (!first_update_done_) {
            last_x_ = robot_x_;
            last_y_ = robot_y_;
        }

        map_memory_.integrateCostmap(latest_costmap_, robot_x_, robot_y_, robot_yaw_);
        //update everything to wait for next update
        should_update_map_ = false;
        costmap_updated_ = false;
        first_update_done_ = true;
    }

    if (!first_update_done_) return;

    //publishing the updated map on the /map topic
    auto& map = map_memory_.getMap();
    map.header.stamp = this->now();
    map_pub_->publish(map);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
