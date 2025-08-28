#include "odometry.h"
#include <cmath>
#include <vector>

using namespace std;

MotionCommand Odometry::computeCommands(vector<pair<int, int>> &path) {
    MotionCommand res = {0.0, 0.0}; // total time (seconds), total angle (degrees)

    if (path.size() < 2)
        return res;  // no movement if path too short

    double total_time = 0.0;
    double total_angle_change = 0.0;

    // Compute initial heading angle from first segment
    double prev_angle = angle(path[0].first, path[0].second, path[1].first, path[1].second);

    for (size_t i = 1; i < path.size(); ++i) {
        int x1 = path[i - 1].first;
        int y1 = path[i - 1].second;
        int x2 = path[i].first;
        int y2 = path[i].second;

        // Distance between two consecutive points
        double dist = distance(x1, y1, x2, y2);
        total_time += dist / linear_vel;  // time = distance / velocity

        // Current segment heading
        double current_angle = angle(x1, y1, x2, y2);

        if (i > 1) {
            // Calculate angle difference between previous and current segment
            double angle_diff = current_angle - prev_angle;

            // Normalize to [-180, 180]
            while (angle_diff > 180) angle_diff -= 360;
            while (angle_diff < -180) angle_diff += 360;

            total_angle_change += fabs(angle_diff);
        }

        prev_angle = current_angle;
    }

    res.time_sec = total_time;
    res.angle_deg = total_angle_change;

    return res;
}
