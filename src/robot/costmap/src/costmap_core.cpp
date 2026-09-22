#include <algorithm>
#include <cmath>

#include "costmap_core.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}

void CostmapCore::initializeCostmap(int width, int height, double resolution) {
  width_ = width;
  height_ = height;
  resolution_ = resolution;
  // initializes the grid with every cell free (0)
  grid_.assign(width_ * height_, 0);
}

//returns weather or not a cell is within the map bounds
bool CostmapCore::insideMap(int gx, int gy) const {
  return gx >= 0 && gx < width_ && gy >= 0 && gy < height_;
}

//makes a certain cell marked as occupied (100), checks if its within the bounds first
void CostmapCore::markObstacle(int gx, int gy) {
  if (insideMap(gx, gy)) {
    grid_[gy * width_ + gx] = 100;
  }
}

void CostmapCore::inflateObstacles(double inflation_radius, int8_t max_cost) {
  const int radius_cells = static_cast<int>(inflation_radius / resolution_);

  
  std::vector<int8_t> inflated = grid_;

  //loops through every cell
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      // don't handle anything that is not marked as occupied
      if (grid_[y * width_ + x] < 100) {
        continue;
      }
      //loops through all cells within the inflation radius
      for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
          const int nx = x + dx;
          const int ny = y + dy;
          //don't handle anything outside map bounds or outside inflation radius
          if (!insideMap(nx, ny)) {
            continue;
          }
          const double dist = std::hypot(dx, dy) * resolution_;
          if (dist > inflation_radius) {
            continue;
          }
          //closer ones get higher cost and farther ones get lower ones
          const int8_t cost = static_cast<int8_t>(max_cost * (1.0 - dist / inflation_radius));
          int8_t& cell = inflated[ny * width_ + nx];
          cell = std::max(cell, cost);
        }
      }
    }
  }

  grid_ = std::move(inflated);
}

}
