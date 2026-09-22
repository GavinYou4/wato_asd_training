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
  //get map data
  int w = map.info.width;
  int h = map.info.height;
  double res = map.info.resolution;
  double ox = map.info.origin.position.x;
  double oy = map.info.origin.position.y;

  //convert start and goal to map indices
  CellIndex start((int)((start_x - ox) / res), (int)((start_y - oy) / res));
  CellIndex goal((int)((goal_x - ox) / res), (int)((goal_y - oy) / res));

  //rejects start/end points if they are they are off the map
  if (start.x < 0 || start.x >= w || start.y < 0 || start.y >= h) {
    RCLCPP_WARN(logger_, "start is off the map");
    return false;
  }
  if (goal.x < 0 || goal.x >= w || goal.y < 0 || goal.y >= h) {
    RCLCPP_WARN(logger_, "goal is off the map");
    return false;
  }

  //rejects goal points if they are in an obstacle
  if (map.data[goal.y * w + goal.x] >= 50) {
    RCLCPP_WARN(logger_, "goal is inside an obstacle");
    return false;
  }

  bool start_blocked = map.data[start.y * w + start.x] >= 50;

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open;
  std::unordered_map<CellIndex, double, CellIndexHash> gscore;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;

  // calculates euclidian distance from a cellindex to the goal cell index
  auto heur = [&](const CellIndex& a) {
    double dx = a.x - goal.x;
    double dy = a.y - goal.y;
    return std::sqrt(dx * dx + dy * dy);
  };

  gscore[start] = 0.0;
  //open is priority queue of cells left to explore sorted by lowest estimated cost
  open.push(AStarNode(start, heur(start)));

  //checks 8 neighbors: up down left right, diagonals
  int dxs[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  int dys[8] = {0, 0, 1, -1, 1, -1, 1, -1};

  bool found = false;
  while (!open.empty()) {
    //gets and removes first cell at top of queue
    AStarNode cur = open.top();
    open.pop();

    //stops search if current cell is the goal
    if (cur.index == goal) {
      found = true;
      break;
    }
    //explores all 8 neighbours listed above of curent cell
    for (int k = 0; k < 8; k++) {
      int nx = cur.index.x + dxs[k];
      int ny = cur.index.y + dys[k];
      //ignores neighbours that are out of bounds
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;

      //gets the cost value of neihbor cell
      int8_t v = map.data[ny * w + nx];
      //avoids all cells with occupancy value over 50
      int limit = 50;
      //if the start already has high occupancy value, the limit within a 15 cell radius is 100 (avoids certain obstacles but gives slack to leave the area)
      if (start_blocked) {
        double sx = nx - start.x;
        double sy = ny - start.y;
        if (sx * sx + sy * sy < 225.0) limit = 100;
      }
      //reject occupied (or fairly occupied) cells
      if (v >= limit) continue;
      //up down left right steps have a cost of 1, diagonal steps (longer) have a cost/distance of root 2
      double step = k < 4 ? 1.0 : 1.4142;
      //if this neighbor cell has a cost value, the step cost is increased. free neighbours are favoured (no penalty added) 
      if (v > 0) step += v * 0.1;
      //creates neighbour cell index and calculates g score which is accumulated cost from start to this cell
      CellIndex nb(nx, ny);
      double g = gscore[cur.index] + step;
      //checks if this neighbor has been seen before.
      auto it = gscore.find(nb);
      //if it is ^^ OR if this way is cheaper than old route
      if (it == gscore.end() || g < it->second) {
        //stores cheapest g score to arrive at this cell, stores parent cell to reconstruct path, adds this neighbour to the priority queue with its estimated total cost
        gscore[nb] = g;
        came_from[nb] = cur.index;
        open.push(AStarNode(nb, g + heur(nb)));
      }
    }
  }
  //once all cells have been explored, if the goal wasn't found, the algorithm failed
  if (!found) return false;


  //reconstructing path
  std::vector<CellIndex> cells;
  CellIndex c = goal;
  while (c != start) {
    cells.push_back(c);
    c = came_from[c];
  }
  cells.push_back(start);
  std::reverse(cells.begin(), cells.end());

  //converts the path from cell indices to world coordinates and packages them into a path message
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
