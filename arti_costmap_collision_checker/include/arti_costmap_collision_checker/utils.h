#ifndef ARTI_COSTMAP_COLLISION_CHECKER_UTILS_H
#define ARTI_COSTMAP_COLLISION_CHECKER_UTILS_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>

namespace arti_costmap_collision_checker
{
namespace utils
{
geometry_msgs::Pose2D toPose2D(const geometry_msgs::Pose& pose);

geometry_msgs::Pose toPose(const geometry_msgs::Pose2D& pose_2d);

visualization_msgs::Marker makeCircleMarker(
  const std::string& frame_id, double center_x, double center_y, double radius);

std_msgs::Bool makeBoolMessage(bool data);
}
}

#endif //ARTI_COSTMAP_COLLISION_CHECKER_UTILS_H
