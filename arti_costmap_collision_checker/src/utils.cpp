#include <arti_costmap_collision_checker/utils.h>
#include <tf/transform_datatypes.h>

namespace arti_costmap_collision_checker
{
namespace utils
{
geometry_msgs::Pose2D toPose2D(const geometry_msgs::Pose& pose)
{
  geometry_msgs::Pose2D pose_2d;
  pose_2d.x = pose.position.x;
  pose_2d.y = pose.position.y;
  pose_2d.theta = tf::getYaw(pose.orientation);
  return pose_2d;
}

geometry_msgs::Pose toPose(const geometry_msgs::Pose2D& pose_2d)
{
  geometry_msgs::Pose pose;
  pose.position.x = pose_2d.x;
  pose.position.y = pose_2d.y;
  pose.position.z = 0.0;
  pose.orientation = tf::createQuaternionMsgFromYaw(pose_2d.theta);
  return pose;
}

visualization_msgs::Marker makeCircleMarker(
  const std::string& frame_id, const double center_x, const double center_y, const double radius)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.color.r = 1;
  marker.color.a = 1;
  marker.scale.x = 0.05; // Line width

  const int NUM_POINTS = 72;
  marker.points.reserve(NUM_POINTS + 1);

  for (int i = 0; i <= NUM_POINTS; ++i)
  {
    const double angle = i * 2.0 * M_PI / NUM_POINTS;
    geometry_msgs::Point point;
    point.x = center_x + radius * std::cos(angle);
    point.y = center_y + radius * std::sin(angle);
    point.z = 0;
    marker.points.push_back(point);
  }
  return marker;
}

std_msgs::Bool makeBoolMessage(const bool data)
{
  std_msgs::Bool message;
  message.data = data ? 1 : 0;
  return message;
}
}
}
