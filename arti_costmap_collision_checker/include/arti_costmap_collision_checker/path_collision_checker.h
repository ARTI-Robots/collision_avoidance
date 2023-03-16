#ifndef ARTI_COSTMAP_COLLISION_CHECKER_PATH_COLLISION_CHECKER_H
#define ARTI_COSTMAP_COLLISION_CHECKER_PATH_COLLISION_CHECKER_H

#include <arti_costmap_collision_checker/costmap_collision_check.h>
#include <nav_msgs/Path.h>

namespace arti_costmap_collision_checker
{
class PathCollisionChecker
{
public:
#if ROS_VERSION_MINIMUM(1, 14, 0) // if MELODIC (v14)
  PathCollisionChecker(const ros::NodeHandle& nh, const std::string &costmap_name, tf2_ros::Buffer& tf_listener);
#else // if KINETIC (v12)
  PathCollisionChecker(const ros::NodeHandle& nh, const std::string &costmap_name, tf::TransformListener& tf_listener);
#endif

  void startCostmaps();
  bool checkPath(const nav_msgs::Path& path);

private:
  bool checkPathInternal(const nav_msgs::Path& path);
  bool isInCollision(const geometry_msgs::Pose& pose, const ros::Time& time_point);
  bool updateTimeForCostmap(const ros::Time& time_point);

#if ROS_VERSION_MINIMUM(1, 14, 0) // if MELODIC (v14)
  tf2_ros::Buffer& tf_listener_;
#else // if KINETIC (v12)
  geometry_msgs::PoseStamped transformPose(const geometry_msgs::PoseStamped& pose, const std_msgs::Header& header);
  tf::TransformListener& tf_listener_;
#endif

  costmap_2d::Costmap2DROS costmap_path_;
  CostmapCollisionCheck collision_checker_path_;
};
}

#endif //ARTI_COSTMAP_COLLISION_CHECKER_PATH_COLLISION_CHECKER_H
