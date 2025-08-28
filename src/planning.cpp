#include <queue>
#include <unordered_map>
#include <algorithm>  // for reverse

vector<pair<int, int>> Planner::pathplanning(pair<int, int> start,
                                             pair<int, int> goal) {
  vector<pair<int, int>> path;

  // Directions: up, down, left, right
  vector<pair<int, int>> directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

  // Priority queue: stores pairs of (f_cost, (x,y))
  // f_cost = g_cost + heuristic
  priority_queue<
      pair<double, pair<int, int>>,
      vector<pair<double, pair<int, int>>>,
      greater<pair<double, pair<int, int>>>>
      open_set;

  // Visited cells
  vector<vector<bool>> visited(rows, vector<bool>(cols, false));

  // For tracing path back
  unordered_map<int, pair<int, int>> came_from;

  // Cost from start to each cell (g_cost)
  vector<vector<double>> g_cost(rows, vector<double>(cols, INFINITY));

  int start_x = start.first, start_y = start.second;
  int goal_x = goal.first, goal_y = goal.second;

  // Initialize start cell
  g_cost[start_x][start_y] = 0.0;
  open_set.push({heuristic(start_x, start_y, goal_x, goal_y), start});

  while (!open_set.empty()) {
    auto current = open_set.top().second;
    open_set.pop();

    int cx = current.first;
    int cy = current.second;

    if (visited[cx][cy])
      continue;
    visited[cx][cy] = true;

    // Goal reached
    if (cx == goal_x && cy == goal_y) {
      // Reconstruct path from goal to start
      pair<int, int> step = goal;
      while (!(step.first == start_x && step.second == start_y)) {
        path.push_back(step);
        int key = step.first * cols + step.second;
        step = came_from[key];
      }
      path.push_back(start);
      std::reverse(path.begin(), path.end());
      return path;
    }

    // Explore neighbors
    for (auto dir : directions) {
      int nx = cx + dir.first;
      int ny = cy + dir.second;

      if (!isvalid(nx, ny) || visited[nx][ny])
        continue;

      double tentative_g = g_cost[cx][cy] + 1.0; // cost to move

      if (tentative_g < g_cost[nx][ny]) {
        g_cost[nx][ny] = tentative_g;
        double f_cost = tentative_g + heuristic(nx, ny, goal_x, goal_y);
        open_set.push({f_cost, {nx, ny}});
        came_from[nx * cols + ny] = {cx, cy};
      }
    }
  }

  // No path found, return empty path
  return path;
}
