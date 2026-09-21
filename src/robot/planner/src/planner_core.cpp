#include <algorithm>
#include <cmath>

#include "planner_core.hpp"

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger)
: logger_(logger) {}

bool PlannerCore::planPath(const nav_msgs::msg::OccupancyGrid& map,
    double start_x, double start_y,
    double goal_x, double goal_y,
    nav_msgs::msg::Path& path)
{
  int w = map.info.width;
  int h = map.info.height;
  double res = map.info.resolution;
  double ox = map.info.origin.position.x;
  double oy = map.info.origin.position.y;

  CellIndex start((int)((start_x - ox) / res), (int)((start_y - oy) / res));
  CellIndex goal((int)((goal_x - ox) / res), (int)((goal_y - oy) / res));

  if (start.x < 0 || start.x >= w || start.y < 0 || start.y >= h) {
    RCLCPP_WARN(logger_, "start is off the map");
    return false;
  }
  if (goal.x < 0 || goal.x >= w || goal.y < 0 || goal.y >= h) {
    RCLCPP_WARN(logger_, "goal is off the map");
    return false;
  }

  if (map.data[goal.y * w + goal.x] >= 50) {
    RCLCPP_WARN(logger_, "goal is inside an obstacle");
    return false;
  }

  bool start_blocked = map.data[start.y * w + start.x] >= 50;

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open;
  std::unordered_map<CellIndex, double, CellIndexHash> gscore;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;

  auto heur = [&](const CellIndex& a) {
    double dx = a.x - goal.x;
    double dy = a.y - goal.y;
    return std::sqrt(dx * dx + dy * dy);
  };

  gscore[start] = 0.0;
  open.push(AStarNode(start, heur(start)));

  int dxs[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  int dys[8] = {0, 0, 1, -1, 1, -1, 1, -1};

  bool found = false;
  while (!open.empty()) {
    AStarNode cur = open.top();
    open.pop();

    if (cur.index == goal) {
      found = true;
      break;
    }

    for (int k = 0; k < 8; k++) {
      int nx = cur.index.x + dxs[k];
      int ny = cur.index.y + dys[k];
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;

      int8_t v = map.data[ny * w + nx];
      int limit = 50;
      if (start_blocked) {
        double sx = nx - start.x;
        double sy = ny - start.y;
        if (sx * sx + sy * sy < 225.0) limit = 100;
      }
      if (v >= limit) continue;

      double step = k < 4 ? 1.0 : 1.4142;
      if (v > 0) step += v * 0.1;

      CellIndex nb(nx, ny);
      double g = gscore[cur.index] + step;
      auto it = gscore.find(nb);
      if (it == gscore.end() || g < it->second) {
        gscore[nb] = g;
        came_from[nb] = cur.index;
        open.push(AStarNode(nb, g + heur(nb)));
      }
    }
  }

  if (!found) return false;

  std::vector<CellIndex> cells;
  CellIndex c = goal;
  while (c != start) {
    cells.push_back(c);
    c = came_from[c];
  }
  cells.push_back(start);
  std::reverse(cells.begin(), cells.end());

  path.poses.clear();
  for (size_t i = 0; i < cells.size(); i++) {
    geometry_msgs::msg::PoseStamped p;
    p.header = path.header;
    p.pose.position.x = ox + (cells[i].x + 0.5) * res;
    p.pose.position.y = oy + (cells[i].y + 0.5) * res;
    p.pose.orientation.w = 1.0;
    path.poses.push_back(p);
  }

  return true;
}

}
