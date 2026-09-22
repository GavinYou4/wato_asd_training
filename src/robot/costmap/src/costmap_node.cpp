#include <cmath>
#include <memory>

#include "costmap_node.hpp"

//0.1m per cell, 30m x 30m costmap
const double resolution = 0.1;        
const int width = 300;                
const int height = 300;               
const double inflation_radius = 1.5;   
const int8_t max_cost = 100;

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    costmap_.initializeCostmap(width, height, resolution);
    for (size_t i = 0; i < scan->ranges.size(); i++)
    {
        double range = scan->ranges[i];
        // ignore invalid ranges
        if (range < scan->range_min || range > scan->range_max || !std::isfinite(range))
            continue;
        // converting polar coordinates to cartesian coordinates
        double angle = scan->angle_min + i * scan->angle_increment;
        double x = range * std::cos(angle);
        double y = range * std::sin(angle);

        //converts the coordinates relative to robot's position to relative to costmap and its resolution
        int gx = static_cast<int>(x / resolution + width / 2);
        int gy = static_cast<int>(y / resolution + height / 2);

        //marks obstacles that were detected by laser
        costmap_.markObstacle(gx, gy);
    }
    //inflate the cells
    costmap_.inflateObstacles(inflation_radius, max_cost);

    publishCostmap(scan->header);
}

void CostmapNode::publishCostmap(const std_msgs::msg::Header& scan_header)
{
    nav_msgs::msg::OccupancyGrid msg;
    msg.header = scan_header;

    msg.info.resolution = resolution;
    msg.info.width = width;
    msg.info.height = height;
    //making the occupancy grid centered around the robot's position
    msg.info.origin.position.x = -(width * resolution) / 2.0;
    msg.info.origin.position.y = -(height * resolution) / 2.0;
    msg.info.origin.orientation.w = 1.0;

    msg.data = costmap_.data();

    costmap_pub_->publish(msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
