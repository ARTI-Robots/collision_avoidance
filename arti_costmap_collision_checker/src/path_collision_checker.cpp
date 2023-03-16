#include <arti_costmap_collision_checker/path_collision_checker.h>
#include <arti_dynamic_costmap/time_dependent_layer.h>
#include <arti_costmap_layer/basic_layer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace arti_costmap_collision_checker
{
#if ROS_VERSION_MINIMUM(1, 14, 0) // if MELODIC (v14)
PathCollisionChecker::PathCollisionChecker(
  const ros::NodeHandle& nh, const std::string& costmap_name, tf2_ros::Buffer& tf_listener)
  : tf_listener_(tf_listener), costmap_path_(costmap_name, tf_listener_), collision_checker_path_(nh, &costmap_path_)
#else // if KINETIC (v12)
PathCollisionChecker::PathCollisionChecker(
  const ros::NodeHandle& nh, const std::string& costmap_name, tf::TransformListener& tf_listener)
  : tf_listener_(tf_listener), costmap_path_(costmap_name, tf_listener_), collision_checker_path_(nh, &costmap_path_)
#endif
{
  costmap_path_.stop();
}

void PathCollisionChecker::startCostmaps()
{
  costmap_path_.start();
}

bool PathCollisionChecker::checkPath(const nav_msgs::Path& path)
{
  bool result = checkPathInternal(path);

  collision_checker_path_.publishCheckedPoses();
  collision_checker_path_.publishPathPoses();
  collision_checker_path_.publishBresenhamCirclePoints();

  return result;
}

bool PathCollisionChecker::checkPathInternal(const nav_msgs::Path& path)
{
#if ROS_VERSION_MINIMUM(1, 14, 0) // if MELODIC (v14)
  geometry_msgs::TransformStamped transform;

  try
  {
    transform = tf_listener_.lookupTransform(costmap_path_.getGlobalFrameID(), path.header.frame_id, path.header.stamp, ros::Duration(1.0));
  }
  catch (tf2::TransformException &ex)
  {
    throw std::runtime_error(
      "failed to get transform between '" + costmap_path_.getGlobalFrameID() + "' and '" + path.header.frame_id
      + "'");
  }

  for (const geometry_msgs::PoseStamped& pose : path.poses)
  {
    geometry_msgs::PoseStamped transformed_pose;
    tf2::doTransform(pose, transformed_pose, transform);

    if (isInCollision(transformed_pose.pose, transformed_pose.header.stamp))
    {
      return false;
    }
  }
#else // if KINETIC (v12)
  if (!tf_listener_.waitForTransform(
    costmap_path_.getGlobalFrameID(), path.header.frame_id, path.header.stamp, ros::Duration(1.0)))
  {
    throw std::runtime_error(
      "failed to get transform between '" + costmap_path_.getGlobalFrameID() + "' and '" + path.header.frame_id
      + "'");
  }

  for (const geometry_msgs::PoseStamped& pose : path.poses)
  {
    geometry_msgs::PoseStamped transformed_pose = transformPose(pose, path.header);

    if (isInCollision(transformed_pose.pose, transformed_pose.header.stamp))
    {
      return false;
    }
  }
#endif

  return true;
}

#if !ROS_VERSION_MINIMUM(1, 14, 0) // if MELODIC (v14)
geometry_msgs::PoseStamped PathCollisionChecker::transformPose(
  const geometry_msgs::PoseStamped& pose, const std_msgs::Header& header)
{
  geometry_msgs::PoseStamped result;

  geometry_msgs::PoseStamped tmp_pose = pose;
  tmp_pose.header = header;
  tf_listener_.transformPose(costmap_path_.getGlobalFrameID(), tmp_pose, result);

  result.header.stamp = pose.header.stamp;

  return result;
}
#endif

bool PathCollisionChecker::isInCollision(const geometry_msgs::Pose& pose, const ros::Time& time_point)
{
  if (!updateTimeForCostmap(time_point))
  {
    return false;
  }

  costmap_path_.updateMap();
  return collision_checker_path_.isInCollision(pose);
}

bool PathCollisionChecker::updateTimeForCostmap(const ros::Time& time_point)
{
  costmap_2d::LayeredCostmap* layered_costmap = costmap_path_.getLayeredCostmap();

  if (!layered_costmap)
  {
    ROS_ERROR("can not retrieve layered costmap");
    return false;
  }

  std::vector<boost::shared_ptr<costmap_2d::Layer> >* costmap_layers = layered_costmap->getPlugins();

  if (!costmap_layers)
  {
    ROS_ERROR("can not retrieve layers costmap");
    return false;
  }

  for (const auto& layer : *costmap_layers)
  {
    boost::shared_ptr<arti_dynamic_costmap::TimeDependentLayer> casted_layer =
      boost::dynamic_pointer_cast<arti_dynamic_costmap::TimeDependentLayer>(layer);
    if (casted_layer)
    {
      casted_layer->useCostmapAt(time_point);
    }

    boost::shared_ptr<arti_costmap_layer::BasicLayer> casted_base_layer =
      boost::dynamic_pointer_cast<arti_costmap_layer::BasicLayer>(layer);
    if (casted_base_layer)
    {
      if (casted_base_layer->hasTransformationError())
      {
        return false;
      }
    }
  }

  return true;
}
}
