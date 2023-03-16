#ifndef ARTI_COSTMAP_COLLISION_CHECKER_COSTMAP_COLLISION_CHECK_H
#define ARTI_COSTMAP_COLLISION_CHECKER_COSTMAP_COLLISION_CHECK_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <arti_nav_core_msgs/Twist2DWithLimits.h>
#include <arti_nav_core_msgs/Movement2DWithLimits.h>

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

#include <costmap_2d/costmap_2d_publisher.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

namespace arti_costmap_collision_checker
{
class CostmapCollisionCheck
{
public:
  CostmapCollisionCheck(const ros::NodeHandle& nh, costmap_2d::Costmap2DROS* costmap);

  virtual ~CostmapCollisionCheck() = default;

  bool isMovementInCollision(const geometry_msgs::Twist& cmd_vel, double sim_time);

  bool isMovementInCollision(
    const geometry_msgs::Pose& start_pose, const geometry_msgs::Twist& cmd_vel, double sim_time);

  bool isMovementInCollision(
    const geometry_msgs::Pose& start_pose, const geometry_msgs::Twist& cmd_vel, double sim_time,
    geometry_msgs::Pose& resulting_pose);

  bool isMovementInCollision(const arti_nav_core_msgs::Twist2DWithLimits& cmd_vel, double sim_time);

  bool isInCollision();

  bool isInCollision(const geometry_msgs::Pose& pose);

  bool isInCollision(const arti_nav_core_msgs::Movement2DWithLimits& movement);

  void publishCheckedPoses();

  void publishPathPoses();

  void publishBresenhamCirclePoints();

  void setPadding(double padding);

private:
  bool isMovementInCollision(
    const geometry_msgs::Pose2D& start_pose, const geometry_msgs::Twist& cmd_vel, double sim_time,
    geometry_msgs::Pose2D& resulting_pose);

  double computeRotationYawStep() const;

  bool isRotationInCollision(
    const geometry_msgs::Pose2D& start_pose, double omega, double sim_time, geometry_msgs::Pose2D& resulting_pose);

  bool isLineInCollision(
    const geometry_msgs::Pose2D& start_pose, double v, double sim_time, geometry_msgs::Pose2D& resulting_pose);

  bool isArcInCollision(
    const geometry_msgs::Pose2D& start_pose, double v, double omega, double sim_time,
    geometry_msgs::Pose2D& resulting_pose);

  bool isLineInCollision(
    const geometry_msgs::Pose2D& start_pose, double vx, double vy, double omega, double sim_time,
    geometry_msgs::Pose2D& resulting_pose);

  bool isInCollisionArcPoint(
    int mx, int dx, int my, int dy, double start_yaw, double delta_yaw, bool center_is_right);

  bool isYawWithinBounds(double yaw, double start_yaw, double delta_yaw);

  bool isCellPoseInCollision(int x, int y, double yaw);

  bool isPoseInCollision(const geometry_msgs::Pose2D& pose);

  bool isPoseInCollision(double x, double y, double yaw);

  unsigned char getCostSafely(unsigned int x, unsigned int y);

  bool checkCostmapForErrors();

  ros::NodeHandle nh_;

  costmap_2d::Costmap2DROS* costmap_;

  /**
   * Minimum angle to rotate robot footprint so that at least one cell of the projection onto the costmap changes.
   */
  double rotation_yaw_step_;

  std::vector<geometry_msgs::Pose> valid_checked_poses_;

  ros::Publisher valid_checked_poses_pub_;

  std::vector<geometry_msgs::Pose> invalid_checked_poses_;

  ros::Publisher invalid_checked_poses_pub_;

  std::vector<geometry_msgs::Pose> path_poses_;

  ros::Publisher path_poses_pub_;

  std::vector<geometry_msgs::Pose> bresenham_circle_points_;

  ros::Publisher bresenham_circle_points_pub_;

  std::vector<visualization_msgs::Marker> circle_markers_;

  ros::Publisher circle_markers_pub_;

  ros::Publisher checked_poses_with_footprint_pub_;
  size_t old_checked_poses_with_footprint_markers_;

  double padding_ = 0.;
};
}

#endif //ARTI_COSTMAP_COLLISION_CHECKER_COSTMAP_COLLISION_CHECK_H
