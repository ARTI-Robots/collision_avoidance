#include <arti_costmap_collision_checker/interactive_path_test_node.h>
#include <angles/angles.h>
#include <arti_collision_avoidance_msgs/CheckPath.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_datatypes.h>

namespace arti_costmap_collision_checker
{
InteractivePathTestNode::InteractivePathTestNode(const ros::NodeHandle& node_handle)
  : node_handle_(node_handle)
{
  collision_checker_ = node_handle_.param<std::string>("collision_checker", "path");
  // Connect to rviz' "Nav Goal" button:
  goal_pose_subscriber_
    = node_handle_.subscribe("/move_base_simple/goal", 1, &InteractivePathTestNode::processGoalPose, this);

  check_path_service_client_ = node_handle_.serviceClient<arti_collision_avoidance_msgs::CheckPath>("check_path");

  do
  {
    ROS_INFO_STREAM("Waiting for service '" << check_path_service_client_.getService() << "'...");
  }
  while (!check_path_service_client_.waitForExistence(ros::Duration(10.0)));
  ROS_INFO_STREAM("Connected to service '" << check_path_service_client_.getService() << "'.");
}

void InteractivePathTestNode::processGoalPose(const geometry_msgs::PoseStampedConstPtr& goal_pose)
{
  const std::string base_frame = "base_link";

  if (!tf_listener_.waitForTransform(goal_pose->header.frame_id, base_frame, goal_pose->header.stamp,
                                     ros::Duration(1.0)))
  {
    ROS_ERROR_STREAM("Timed out waiting for transform between '"
                       << goal_pose->header.frame_id << "' and '" << base_frame << "'");
    return;
  }

  geometry_msgs::PoseStamped start_pose;
  try
  {
    geometry_msgs::PoseStamped base_pose;
    base_pose.header.stamp = goal_pose->header.stamp;
    base_pose.header.frame_id = base_frame;
    base_pose.pose.orientation.w = 1.0;
    tf_listener_.transformPose(goal_pose->header.frame_id, base_pose, start_pose);
  }
  catch (const tf::TransformException& e)
  {
    ROS_ERROR_STREAM("Failed to get transform between '"
                       << goal_pose->header.frame_id << "' and '" << base_frame << "': " << e.what());
    return;
  }

  arti_collision_avoidance_msgs::CheckPathRequest request;
  request.path.header = goal_pose->header;
  generatePath(goal_pose->header, start_pose.pose, goal_pose->pose, request.path.poses);
  request.collision_checker = collision_checker_;
  arti_collision_avoidance_msgs::CheckPathResponse response;
  if (check_path_service_client_.call(request, response))
  {
    ROS_INFO_STREAM("Service '" << check_path_service_client_.getService() << "' returned ok: " << (response.ok != 0));
  }
  else
  {
    ROS_ERROR_STREAM("Call to service '" << check_path_service_client_.getService() << "' failed");
  }
}

void InteractivePathTestNode::generatePath(
  const std_msgs::Header& header, const geometry_msgs::Pose& start_pose,
  const geometry_msgs::Pose& goal_pose, std::vector<geometry_msgs::PoseStamped>& poses) const
{
  const double sim_time = 2.0;
  const int sim_steps = 10;

  geometry_msgs::Pose2D start_pose_2d;
  start_pose_2d.x = start_pose.position.x;
  start_pose_2d.y = start_pose.position.y;
  start_pose_2d.theta = tf::getYaw(start_pose.orientation);

  geometry_msgs::Pose2D delta_pose;
  delta_pose.x = goal_pose.position.x - start_pose_2d.x;
  delta_pose.y = goal_pose.position.y - start_pose_2d.y;
  delta_pose.theta = angles::normalize_angle(tf::getYaw(goal_pose.orientation) - start_pose_2d.theta);

  poses.reserve(sim_steps + 1);
  for (int i = 0; i <= sim_steps; ++i)
  {
    geometry_msgs::PoseStamped pose;
    pose.header = header;
    pose.header.stamp += ros::Duration(sim_time * i / sim_steps);
    pose.pose.position.x = start_pose_2d.x + delta_pose.x * i / sim_steps;
    pose.pose.position.y = start_pose_2d.y + delta_pose.y * i / sim_steps;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(start_pose_2d.theta + delta_pose.theta * i / sim_steps);
    poses.push_back(pose);
  }
}
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "interactive_path_test");

  ros::NodeHandle nh("~");

  arti_costmap_collision_checker::InteractivePathTestNode node(nh);

  ros::spin();
  return 0;
}
