#ifndef ARTI_COSTMAP_COLLISION_CHECKER_COLLISION_CHECKER_NODE_H
#define ARTI_COSTMAP_COLLISION_CHECKER_COLLISION_CHECKER_NODE_H

#include <arti_collision_avoidance_msgs/CheckPath.h>
#include <arti_collision_avoidance_msgs/ChangeState.h>
#include <arti_collision_avoidance_msgs/State.h>
#include <arti_costmap_collision_checker/costmap_collision_check.h>
#include <arti_costmap_collision_checker/CollisionCheckerNodeConfig.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Twist.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/service_server.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <arti_costmap_collision_checker/path_collision_checker.h>
#include <map>
#include <thread>
#include <atomic>
#include <mutex>

namespace arti_costmap_collision_checker
{
class CollisionCheckerNode
{
public:
  explicit CollisionCheckerNode(const ros::NodeHandle& nh);

protected:
  void processCommand(const geometry_msgs::Twist::ConstPtr& cmd_msg);
  bool checkCommandCostmapForErrors();

  bool checkPath(
    arti_collision_avoidance_msgs::CheckPath::Request& req,
    arti_collision_avoidance_msgs::CheckPath::Response& res);

  bool changeState(
    arti_collision_avoidance_msgs::ChangeState::Request& req,
    arti_collision_avoidance_msgs::ChangeState::Response& res);

  void reconfigure(CollisionCheckerNodeConfig& config);
  void stateTimerCB(const ros::TimerEvent&);

  void startCostmaps();

  ros::NodeHandle nh_;

#if ROS_VERSION_MINIMUM(1, 14, 0) // if MELODIC (v14)
  tf2_ros::Buffer tf_listener_;
#else // if KINETIC (v12)
  tf::TransformListener tf_listener_;
#endif

  std::mutex collision_checker_cmd_mutex_;
  std::shared_ptr<costmap_2d::Costmap2DROS> costmap_cmd_;
  std::unique_ptr<CostmapCollisionCheck> collision_checker_cmd_;
  std::mutex collision_checkers_path_mutex_;
  std::map<std::string, std::unique_ptr<PathCollisionChecker>> collision_checkers_path_;

  arti_collision_avoidance_msgs::State state_;

  dynamic_reconfigure::Server<CollisionCheckerNodeConfig> config_server_;
  CollisionCheckerNodeConfig config_;

  ros::Publisher cmd_max_pub_;
  ros::Publisher state_pub_;
  ros::Publisher pose_in_collision_pub_;
  ros::Publisher cmd_in_collision_pub_;
  ros::Publisher path_in_collision_pub_;
  ros::Subscriber cmd_vel_sub_;
  ros::ServiceServer check_path_service_server_;
  ros::ServiceServer change_state_service_server_;
  ros::Timer state_timer_;
  std::thread start_costmaps_;

  std::atomic<bool> initialized_;
};
}

#endif //ARTI_COSTMAP_COLLISION_CHECKER_COLLISION_CHECKER_NODE_H
