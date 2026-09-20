#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace robot
{

class CostmapCore {
  public:

    explicit CostmapCore(const rclcpp::Logger& logger);

    void initializeCostmap(int width, int height, double resolution);

    bool insideMap(int gx, int gy) const;
    void markObstacle(int gx, int gy);

    void inflateObstacles(double inflation_radius, int8_t max_cost);

    const std::vector<int8_t>& data() const { return grid_; }
    int width() const { return width_; }
    int height() const { return height_; }
    double resolution() const { return resolution_; }

  private:
    rclcpp::Logger logger_;

    int width_ = 0;
    int height_ = 0;
    double resolution_ = 0.1;
    std::vector<int8_t> grid_;
};

}

#endif
