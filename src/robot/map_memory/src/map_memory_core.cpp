#include <cmath>

#include "map_memory_core.hpp"

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger)
  : logger_(logger) {}

void MapMemoryCore::initGlobalMap(int width, int height, double resolution, double origin_x, double origin_y) {
  global_map_.info.width = width;
  global_map_.info.height = height;
  global_map_.info.resolution = resolution;
  global_map_.info.origin.position.x = origin_x;
  global_map_.info.origin.position.y = origin_y;
  global_map_.info.origin.orientation.w = 1.0;
  global_map_.header.frame_id = "sim_world";
  //sets everything in the global map to unkown (-1)
  global_map_.data.assign(width * height, -1);
}

void MapMemoryCore::integrateCostmap(const nav_msgs::msg::OccupancyGrid& costmap, double robot_x, double robot_y, double robot_yaw) {
  double res = costmap.info.resolution;
  double c = std::cos(robot_yaw);
  double s = std::sin(robot_yaw);

  for (unsigned int j = 0; j < costmap.info.height; j++) {
    for (unsigned int i = 0; i < costmap.info.width; i++) {
      int8_t val = costmap.data[j * costmap.info.width + i];
      if (val < 0) continue;

      // costmap cell coordinates in the robots frame
      double lx = costmap.info.origin.position.x + (i + 0.5) * res;
      double ly = costmap.info.origin.position.y + (j + 0.5) * res;

      //translates the costmap coordinates to coordinates on the global map
      double wx = robot_x + c * lx - s * ly;
      double wy = robot_y + s * lx + c * ly;

      //turn those coordinates into global map indices
      int gx = (int)((wx - global_map_.info.origin.position.x) / global_map_.info.resolution);
      int gy = (int)((wy - global_map_.info.origin.position.y) / global_map_.info.resolution);

      //ignore invalid indices
      if (gx < 0 || gy < 0 || gx >= (int)global_map_.info.width || gy >= (int)global_map_.info.height)
        continue;

      int idx = gy * global_map_.info.width + gx;
      // only update the global map cell if the costmal cell is HIGHER
      if (val > global_map_.data[idx])
        global_map_.data[idx] = val;
    }
  }
}

}
