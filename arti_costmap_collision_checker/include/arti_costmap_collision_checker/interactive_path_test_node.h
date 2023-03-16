#ifndef ARTI_COSTMAP_COLLISION_CHECKER_INTERACTIVE_PATH_TEST_NODE_H
#define ARTI_COSTMAP_COLLISION_CHECKER_INTERACTIVE_PATH_TEST_NODE_H

#include <geometry_msgs/PoseStamped.h>
#include <ros/node_handle.h>
#include <ros/service_client.h>
#include <ros/subscriber.h>
#include <tf/transform_listener.h>

namespace arti_costmap_collision_checker
{
class InteractivePathTestNode
{
public:
  explicit InteractivePathTestNode(const ros::NodeHandle& node_handle);

protected:
  void processGoalPose(const geometry_msgs::PoseStampedConstPtr& goal_pose);

  void generatePath(
    const std_msgs::Header& header, const geometry_msgs::Pose& start_pose,
    const geometry_msgs::Pose& goal_pose, std::vector<geometry_msgs::PoseStamped>& poses) const;

  ros::NodeHandle node_handle_;
  ros::Subscriber goal_pose_subscriber_;
  ros::ServiceClient check_path_service_client_;
  tf::TransformListener tf_listener_;
  std::string  collision_checker_;
};
}

#endif //ARTI_COSTMAP_COLLISION_CHECKER_INTERACTIVE_PATH_TEST_NODE_H
