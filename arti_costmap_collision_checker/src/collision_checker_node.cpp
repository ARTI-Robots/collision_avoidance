#include <arti_costmap_collision_checker/collision_checker_node.h>
#include <arti_costmap_collision_checker/utils.h>
#include <functional>
#include <std_msgs/Bool.h>
#include <arti_profiling/duration_measurement.h>
#include <arti_ros_param/arti_ros_param.h>
#include <arti_ros_param/collections.h>
#include <arti_costmap_layer/basic_layer.h>

namespace arti_costmap_collision_checker
{
CollisionCheckerNode::CollisionCheckerNode(const ros::NodeHandle& nh)
  : nh_(nh), config_server_(nh_), initialized_(false)
{
  state_.state = arti_collision_avoidance_msgs::State::STATE_INITIALIZING;

  config_server_.setCallback(std::bind(&CollisionCheckerNode::reconfigure, this, std::placeholders::_1));

  cmd_max_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_max", 1);
  state_pub_ = nh_.advertise<arti_collision_avoidance_msgs::State>("state", 1);
  pose_in_collision_pub_ = nh_.advertise<std_msgs::Bool>("pose_in_collision", 1);
  cmd_in_collision_pub_ = nh_.advertise<std_msgs::Bool>("cmd_in_collision", 1);
  path_in_collision_pub_ = nh_.advertise<std_msgs::Bool>("path_in_collision", 1);

  cmd_vel_sub_ = nh_.subscribe("cmd_vel", 1, &CollisionCheckerNode::processCommand, this);
  check_path_service_server_ = nh_.advertiseService("check_path", &CollisionCheckerNode::checkPath, this);
  change_state_service_server_ = nh_.advertiseService("change_state", &CollisionCheckerNode::changeState, this);

  start_costmaps_ = std::thread(std::bind(&CollisionCheckerNode::startCostmaps, this));
}

void CollisionCheckerNode::processCommand(const geometry_msgs::Twist::ConstPtr& cmd_msg)
{
  geometry_msgs::Twist cmd_to_pub;

  if ((state_.state == arti_collision_avoidance_msgs::State::STATE_INITIALIZING) || (!initialized_))
  {
    // during initalization everything is unsafe
    cmd_max_pub_.publish(cmd_to_pub);
    return;
  }

  std::lock_guard<std::mutex> cmd_lock(collision_checker_cmd_mutex_);
  if (!collision_checker_cmd_)
  {
    ROS_ERROR_STREAM("collision checker still not created after intialization");
    cmd_max_pub_.publish(cmd_to_pub);
    return;
  }

  if (checkCommandCostmapForErrors())
  {
    ROS_ERROR_STREAM("costmap has an error can not perform collision checking");
    state_.state = arti_collision_avoidance_msgs::State::STATE_UNKNOWN;
    cmd_max_pub_.publish(cmd_to_pub);
    return;
  }

  const bool pose_in_collision = collision_checker_cmd_->isInCollision();
  pose_in_collision_pub_.publish(utils::makeBoolMessage(pose_in_collision));

  const bool cmd_in_collision = collision_checker_cmd_->isMovementInCollision(*cmd_msg, config_.sim_time);
  cmd_in_collision_pub_.publish(utils::makeBoolMessage(cmd_in_collision));

  collision_checker_cmd_->publishCheckedPoses();
  collision_checker_cmd_->publishPathPoses();
  collision_checker_cmd_->publishBresenhamCirclePoints();

  if (state_.state == arti_collision_avoidance_msgs::State::STATE_OK
      || state_.state == arti_collision_avoidance_msgs::State::STATE_UNKNOWN)
  {
    if (pose_in_collision)
    {
      state_.state = arti_collision_avoidance_msgs::State::STATE_ROBOT_IN_COLLISION;
    }
    else
    {
      state_.state = arti_collision_avoidance_msgs::State::STATE_OK;

      if (!cmd_in_collision)
      {
        cmd_to_pub = *cmd_msg;
      }
    }
  }

  cmd_max_pub_.publish(cmd_to_pub);
}

bool CollisionCheckerNode::checkCommandCostmapForErrors()
{
  if (!costmap_cmd_)
  {
    return true;
  }

  costmap_2d::LayeredCostmap* layered_costmap = costmap_cmd_->getLayeredCostmap();

  if (!layered_costmap)
  {
    ROS_ERROR("can not retrieve layered costmap");
    return true;
  }

  std::vector<boost::shared_ptr<costmap_2d::Layer> >* costmap_layers = layered_costmap->getPlugins();

  if (!costmap_layers)
  {
    ROS_ERROR("can not retrieve layers costmap");
    return true;
  }

  for (const auto& layer : *costmap_layers)
  {
    boost::shared_ptr<arti_costmap_layer::BasicLayer> casted_base_layer =
      boost::dynamic_pointer_cast<arti_costmap_layer::BasicLayer>(layer);
    if (casted_base_layer)
    {
      if (casted_base_layer->hasTransformationError())
      {
        return true;
      }
    }
  }

  return false;
}

bool CollisionCheckerNode::checkPath(
  arti_collision_avoidance_msgs::CheckPath::Request& req, arti_collision_avoidance_msgs::CheckPath::Response& res)
{
  if ((state_.state == arti_collision_avoidance_msgs::State::STATE_INITIALIZING) || (!initialized_))
  {
    ROS_WARN_STREAM("Ignoring check path collision checker is still initalizing");

    res.ok = false;
    return false;
  }

  std::lock_guard<std::mutex> path_lock(collision_checkers_path_mutex_);

  auto it = collision_checkers_path_.find(req.collision_checker);

  if (it == collision_checkers_path_.end())
  {
    ROS_ERROR_STREAM("specified collision checker '" << req.collision_checker << "' not known");
    return false;
  }

  const bool path_ok = it->second->checkPath(req.path);
  path_in_collision_pub_.publish(utils::makeBoolMessage(!path_ok));

  res.ok = path_ok && (state_.state == arti_collision_avoidance_msgs::State::STATE_OK
                       || state_.state == arti_collision_avoidance_msgs::State::STATE_ROBOT_IN_COLLISION
                       || state_.state == arti_collision_avoidance_msgs::State::STATE_UNKNOWN);

  return true;
}

bool CollisionCheckerNode::changeState(
  arti_collision_avoidance_msgs::ChangeState::Request& req,
  arti_collision_avoidance_msgs::ChangeState::Response& /*res*/)
{
  if (!initialized_)
  {
    ROS_WARN_STREAM("Ignoring state change as collision checker is still initalizing");

    return false;
  }

  if (state_.state == arti_collision_avoidance_msgs::State::STATE_INITIALIZING)
  {
    if (req.event.event == arti_collision_avoidance_msgs::Event::EVENT_SYSTEM_OK)
    {
      state_.state = arti_collision_avoidance_msgs::State::STATE_UNKNOWN;
      return true;
    }
    else if (req.event.event == arti_collision_avoidance_msgs::Event::EVENT_GENERAL_ERROR)
    {
      state_.state = arti_collision_avoidance_msgs::State::STATE_GENERAL_ERROR;
      return true;
    }
  }
  else if (state_.state == arti_collision_avoidance_msgs::State::STATE_UNKNOWN)
  {
    if (req.event.event == arti_collision_avoidance_msgs::Event::EVENT_SYSTEM_OK)
    {
      // ignore event as we are already in state unknwon
      return true;
    }
    else if (req.event.event == arti_collision_avoidance_msgs::Event::EVENT_GENERAL_ERROR)
    {
      state_.state = arti_collision_avoidance_msgs::State::STATE_GENERAL_ERROR;
      return true;
    }
  }
  else if (state_.state == arti_collision_avoidance_msgs::State::STATE_OK)
  {
    if (req.event.event == arti_collision_avoidance_msgs::Event::EVENT_SYSTEM_OK)
    {
      // ignore event as we are already in state unknwon
      return true;
    }
    else if (req.event.event == arti_collision_avoidance_msgs::Event::EVENT_GENERAL_ERROR)
    {
      state_.state = arti_collision_avoidance_msgs::State::STATE_GENERAL_ERROR;
      return true;
    }
  }
  else if (state_.state == arti_collision_avoidance_msgs::State::STATE_ROBOT_IN_COLLISION)
  {
    if (req.event.event == arti_collision_avoidance_msgs::Event::EVENT_SYSTEM_OK)
    {
      // ignore event as we are already in state unknwon
      return true;
    }
    else if (req.event.event == arti_collision_avoidance_msgs::Event::EVENT_GENERAL_ERROR)
    {
      state_.state = arti_collision_avoidance_msgs::State::STATE_GENERAL_ERROR;
      return true;
    }
    else if (req.event.event == arti_collision_avoidance_msgs::Event::EVENT_RESET)
    {
      state_.state = arti_collision_avoidance_msgs::State::STATE_UNKNOWN;
      return true;
    }
  }
  else if (state_.state == arti_collision_avoidance_msgs::State::STATE_GENERAL_ERROR)
  {
    if (req.event.event == arti_collision_avoidance_msgs::Event::EVENT_SYSTEM_OK)
    {
      state_.state = arti_collision_avoidance_msgs::State::STATE_UNKNOWN;
      return true;
    }
    else if (req.event.event == arti_collision_avoidance_msgs::Event::EVENT_GENERAL_ERROR)
    {
      // ignore event as we are already in state general error
      return true;
    }
  }
  else
  {
    throw std::logic_error("current state has unknown value '" + state_.state + "', this should never happen");
  }

  throw std::invalid_argument(
    "tried to change state with event '" + req.event.event + "' which is invalid when in state " + state_.state);
}

void CollisionCheckerNode::reconfigure(CollisionCheckerNodeConfig& config)
{
  if (!state_timer_ || config_.state_pub_duration != config.state_pub_duration)
  {
    if (state_timer_)
    {
      state_timer_.stop();
    }
    state_timer_ = nh_.createTimer(ros::Duration(config.state_pub_duration), &CollisionCheckerNode::stateTimerCB, this);
  }

  config_ = config;
}

void CollisionCheckerNode::stateTimerCB(const ros::TimerEvent& /*event*/)
{
  state_pub_.publish(state_);
}

void CollisionCheckerNode::startCostmaps()
{
  std::unique_lock<std::mutex> cmd_lock(collision_checker_cmd_mutex_);
  costmap_cmd_ = std::make_shared<costmap_2d::Costmap2DROS>("cmd_costmap", tf_listener_);
  collision_checker_cmd_.reset(new CostmapCollisionCheck(ros::NodeHandle(nh_, "cmd"), costmap_cmd_.get()));
  costmap_cmd_->stop();
  cmd_lock.unlock();

  std::vector<std::string> path_collision_checker_names = arti_ros_param::getOptionalParam<std::vector<std::string>>(
  nh_, "path_collision_checker_names", std::vector<std::string>());

  if (path_collision_checker_names.empty())
  {
    path_collision_checker_names.emplace_back("path");
  }

  std::unique_lock<std::mutex> path_lock(collision_checkers_path_mutex_);
  for (const std::string& collision_checker_name : path_collision_checker_names)
  {
    collision_checkers_path_.insert(
      std::make_pair(collision_checker_name,
                     std::unique_ptr<PathCollisionChecker>(
                       new PathCollisionChecker(ros::NodeHandle(nh_, collision_checker_name),
                                                collision_checker_name + "_costmap", tf_listener_))));
  }
  path_lock.unlock();

  cmd_lock.lock();
  costmap_cmd_->start();
  cmd_lock.unlock();

  path_lock.lock();
  for (auto& collision_checker_path : collision_checkers_path_)
  {
    collision_checker_path.second->startCostmaps();
  }
  path_lock.unlock();

  initialized_ = true;

  ROS_INFO_STREAM("done starting costmaps");
}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "collision_checker");
  ros::NodeHandle nh("~");
  arti_costmap_collision_checker::CollisionCheckerNode node(nh);
  ros::spin();
  return 0;
}
