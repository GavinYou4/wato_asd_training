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
  grid_.assign(width_ * height_, 0);
}

bool CostmapCore::insideMap(int gx, int gy) const {
  return gx >= 0 && gx < width_ && gy >= 0 && gy < height_;
}

void CostmapCore::markObstacle(int gx, int gy) {
  if (insideMap(gx, gy)) {
    grid_[gy * width_ + gx] = 100;
  }
}

void CostmapCore::inflateObstacles(double inflation_radius, int8_t max_cost) {
  const int radius_cells = static_cast<int>(inflation_radius / resolution_);

  
  std::vector<int8_t> inflated = grid_;

  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      if (grid_[y * width_ + x] < 100) {
        continue;
      }
      for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
          const int nx = x + dx;
          const int ny = y + dy;
          if (!insideMap(nx, ny)) {
            continue;
          }
          const double dist = std::hypot(dx, dy) * resolution_;
          if (dist > inflation_radius) {
            continue;
          }
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
