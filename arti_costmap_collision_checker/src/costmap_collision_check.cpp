#include <arti_costmap_collision_checker/costmap_collision_check.h>
#include <angles/angles.h>
#include <arti_costmap_collision_checker/utils.h>
#include <geometry_msgs/PoseArray.h>
#include <tug_bresenham/circle_iterator.h>
#include <tug_bresenham/line_iterator.h>
#include <visualization_msgs/MarkerArray.h>
#include <arti_costmap_layer/basic_layer.h>
#include <mutex>

namespace arti_costmap_collision_checker
{
CostmapCollisionCheck::CostmapCollisionCheck(const ros::NodeHandle& nh, costmap_2d::Costmap2DROS* costmap)
  : nh_(nh), costmap_(costmap), rotation_yaw_step_(computeRotationYawStep()),
    old_checked_poses_with_footprint_markers_(0)
{
  valid_checked_poses_pub_ = nh_.advertise<geometry_msgs::PoseArray>("valid_checked_poses", 1, true);

  invalid_checked_poses_pub_ = nh_.advertise<geometry_msgs::PoseArray>("invalid_checked_poses", 1, true);

  checked_poses_with_footprint_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("checked_poses_with_footprint", 1,
                                                                                     true);

  path_poses_pub_ = nh_.advertise<geometry_msgs::PoseArray>("path_poses", 1, true);

  bresenham_circle_points_pub_ = nh_.advertise<geometry_msgs::PoseArray>("bresenham_circle_points", 1, true);

  circle_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("circle_markers", 1, true);
}

double CostmapCollisionCheck::computeRotationYawStep() const
{
  std::unique_lock<costmap_2d::Costmap2D::mutex_t> costmap_lock(*this->costmap_->getCostmap()->getMutex());

  const double footprint_max_radius = costmap_->getLayeredCostmap()->getCircumscribedRadius();
  if (footprint_max_radius > 0.0)
  {
    // Get size of costmap cell in meters:
    const double cell_size = costmap_->getCostmap()->getResolution();
    return 2.0 * std::asin(cell_size / (2.0 * footprint_max_radius));
  }

  ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << ": got invalid robot footprint");
  return 0.1;  // Use some sensible value
}

bool CostmapCollisionCheck::isInCollision()
{
#if ROS_VERSION_MINIMUM(1, 14, 0) // if MELODIC (v14)
  geometry_msgs::PoseStamped pose_stamped;
  {
    std::unique_lock<costmap_2d::Costmap2D::mutex_t> costmap_lock(*this->costmap_->getCostmap()->getMutex());
    if (!costmap_->getRobotPose(pose_stamped))
    {
      ROS_ERROR("can not retrieve robot pose from costmap");
      return false;
    }
  }

  return isPoseInCollision(utils::toPose2D(pose_stamped.pose));
#else // if KINETIC (v12)
  tf::Stamped<tf::Pose> robot_pose;
  {
    std::unique_lock<costmap_2d::Costmap2D::mutex_t> costmap_lock(*this->costmap_->getCostmap()->getMutex());
    if (!costmap_->getRobotPose(robot_pose))
    {
      ROS_ERROR("can not retrieve robot pose from costmap");
      return false;
    }
  }

  geometry_msgs::Pose pose;
  tf::poseTFToMsg(robot_pose, pose);

  return isPoseInCollision(utils::toPose2D(pose));
#endif
}

bool CostmapCollisionCheck::isInCollision(const geometry_msgs::Pose& pose)
{
  return isPoseInCollision(utils::toPose2D(pose));
}

bool CostmapCollisionCheck::isInCollision(const arti_nav_core_msgs::Movement2DWithLimits& movement)
{
  geometry_msgs::Pose pose;
  pose.position.x = movement.pose.point.x.value;
  pose.position.y = movement.pose.point.y.value;

  pose.orientation = tf::createQuaternionMsgFromYaw(movement.pose.theta.value);

  return isInCollision(pose);
}

bool CostmapCollisionCheck::isMovementInCollision(const geometry_msgs::Twist& cmd_vel, double sim_time)
{
#if ROS_VERSION_MINIMUM(1, 14, 0) // if MELODIC (v14)
  geometry_msgs::PoseStamped robot_pose;
  {
    std::unique_lock<costmap_2d::Costmap2D::mutex_t> costmap_lock(*this->costmap_->getCostmap()->getMutex());

    if (!costmap_->getRobotPose(robot_pose))
    {
      ROS_ERROR("can not retrieve robot pose from costmap");
      return false;
    }
  }

  return isMovementInCollision(robot_pose.pose, cmd_vel, sim_time);
#else // if KINETIC (v12)
  tf::Stamped<tf::Pose> robot_pose;
  {
    std::unique_lock<costmap_2d::Costmap2D::mutex_t> costmap_lock(*this->costmap_->getCostmap()->getMutex());
    if (!costmap_->getRobotPose(robot_pose))
    {
      ROS_ERROR("can not retrieve robot pose from costmap");
      return false;
    }
  }

  geometry_msgs::Pose start_pose;
  tf::poseTFToMsg(robot_pose, start_pose);

  return isMovementInCollision(start_pose, cmd_vel, sim_time);
#endif
}

bool CostmapCollisionCheck::isMovementInCollision(
  const geometry_msgs::Pose& start_pose, const geometry_msgs::Twist& cmd_vel, double sim_time)
{
  geometry_msgs::Pose resulting_pose;
  return isMovementInCollision(start_pose, cmd_vel, sim_time, resulting_pose);
}

bool CostmapCollisionCheck::isMovementInCollision(
  const geometry_msgs::Pose& start_pose, const geometry_msgs::Twist& cmd_vel, double sim_time,
  geometry_msgs::Pose& resulting_pose)
{
  geometry_msgs::Pose2D resulting_pose_2d;

  bool result = isMovementInCollision(utils::toPose2D(start_pose), cmd_vel, sim_time, resulting_pose_2d);

  resulting_pose = utils::toPose(resulting_pose_2d);

  return result;
}

bool CostmapCollisionCheck::isMovementInCollision(const arti_nav_core_msgs::Twist2DWithLimits& cmd_vel, double sim_time)
{
  geometry_msgs::Twist used_cmd_vel;
  used_cmd_vel.linear.x = cmd_vel.x.value;
  used_cmd_vel.linear.y = cmd_vel.y.value;
  used_cmd_vel.angular.z = cmd_vel.theta.value;

  return isMovementInCollision(used_cmd_vel, sim_time);
}

bool CostmapCollisionCheck::isMovementInCollision(
  const geometry_msgs::Pose2D& start_pose, const geometry_msgs::Twist& cmd_vel, double sim_time,
  geometry_msgs::Pose2D& resulting_pose)
{
  if (cmd_vel.linear.z != 0 || cmd_vel.angular.x != 0 || cmd_vel.angular.y != 0)
  {
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << ": cmd_vel contains invalid values for 2D motion drive");
  }

  double omega = cmd_vel.angular.z;
  double vx = cmd_vel.linear.x;
  double vy = cmd_vel.linear.y;

  ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ": called");
  if (vy != 0)
  {
    // Check for collision of omni drive
    if ((std::fabs(vx * sim_time) > costmap_->getCostmap()->getResolution())
        || (std::fabs(vy * sim_time) > costmap_->getCostmap()->getResolution()))
    {
      ROS_DEBUG_STREAM("check collision with arc");
      return isLineInCollision(start_pose, vx, vy, omega, sim_time, resulting_pose);
    }

    if ((std::fabs(omega * sim_time) > rotation_yaw_step_))
    {
      ROS_DEBUG_STREAM("check collision with rotation");
      return isRotationInCollision(start_pose, omega, sim_time, resulting_pose);
    }

    ROS_DEBUG_STREAM("check collision with pose");
    return isPoseInCollision(start_pose);
  }

  // The remaining code deals with differential drive:
  ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ": performing collision check for vx: " << vx << " and omega: " << omega);

  // If radius is larger than costmap, assume linear motion (note that the radius computation is rewritten to avoid division by zero):
  const double costmap_size = std::max(costmap_->getCostmap()->getSizeInMetersX(),
                                       costmap_->getCostmap()->getSizeInMetersY());
  if (std::fabs(costmap_size * omega) < std::fabs(vx))
  {
    ROS_DEBUG_STREAM("check collision with line");
    return isLineInCollision(start_pose, vx, sim_time, resulting_pose);
  }

  double radius = std::fabs(vx / omega);
  if (radius > costmap_->getCostmap()->getResolution())
  {
    ROS_DEBUG_STREAM("check collision with arc");
    return isArcInCollision(start_pose, vx, omega, sim_time, resulting_pose);
  }

  if ((std::fabs(omega * sim_time) > rotation_yaw_step_))
  {
    ROS_DEBUG_STREAM("check collision with rotation");
    return isRotationInCollision(start_pose, omega, sim_time, resulting_pose);
  }

  ROS_DEBUG_STREAM("check collision with pose");
  return isPoseInCollision(start_pose);
}

bool CostmapCollisionCheck::isRotationInCollision(
  const geometry_msgs::Pose2D& start_pose, double omega, double sim_time, geometry_msgs::Pose2D& resulting_pose)
{
  ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ": called");
  const double delta_yaw = std::min(std::max(-2.0 * M_PI, omega * sim_time), 2.0 * M_PI);
  const double min_yaw = std::min(start_pose.theta, start_pose.theta + delta_yaw);
  const double max_yaw = std::max(start_pose.theta, start_pose.theta + delta_yaw);
  ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ": min_yaw: " << min_yaw << ", max_yaw: " << max_yaw);

  resulting_pose = start_pose;
  resulting_pose.theta = max_yaw;

  for (int i = 0; i >= 0; ++i)  // This loop only finishes when i overflows, so this should never actually happen!
  {
    const double yaw = min_yaw + i * rotation_yaw_step_;
    if (yaw >= max_yaw)
    {
      // We're finished checking all yaws without detecting a collision, so let's check one last time and we're done:
      return isPoseInCollision(start_pose.x, start_pose.y, max_yaw);
    }
    if (isPoseInCollision(start_pose.x, start_pose.y, yaw))
    {
      return true;
    }
  }
  ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << ": loop finished, this should never happen! This probably means that"
                                          " rotation_yaw_step_ <= 0");
  return false;
}

bool CostmapCollisionCheck::isLineInCollision(
  const geometry_msgs::Pose2D& start_pose, double v, double sim_time, geometry_msgs::Pose2D& resulting_pose)
{
  ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ": called");

  double robot_end_pose_x = start_pose.x + cos(start_pose.theta) * v * sim_time;
  double robot_end_pose_y = start_pose.y + sin(start_pose.theta) * v * sim_time;

  resulting_pose.x = robot_end_pose_x;
  resulting_pose.y = robot_end_pose_y;
  resulting_pose.theta = start_pose.theta;

  int robot_x, robot_y, robot_end_x, robot_end_y;
  {
    std::unique_lock<costmap_2d::Costmap2D::mutex_t> costmap_lock(*this->costmap_->getCostmap()->getMutex());
    costmap_->getCostmap()->worldToMapNoBounds(start_pose.x, start_pose.y, robot_x, robot_y);
    costmap_->getCostmap()->worldToMapNoBounds(robot_end_pose_x, robot_end_pose_y, robot_end_x, robot_end_y);
    ROS_DEBUG_STREAM(
      __PRETTY_FUNCTION__ << ": robot_end_pose: " << robot_end_pose_x << ", " << robot_end_pose_y << " in map "
                          << robot_end_x << ", " << robot_end_y);
    ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ": start checking with robot_x: " << robot_x << " robot_y: " << robot_y
                                         << " robot_end_x: " << robot_end_x << " robot_end_y: " << robot_end_y);
  }

  geometry_msgs::Pose tmp_robot_end_pose = utils::toPose(start_pose);
  {
    std::unique_lock<costmap_2d::Costmap2D::mutex_t> costmap_lock(*this->costmap_->getCostmap()->getMutex());
    costmap_->getCostmap()->mapToWorld(robot_end_x, robot_end_y, tmp_robot_end_pose.position.x,
                                       tmp_robot_end_pose.position.y);
  }
  path_poses_.push_back(tmp_robot_end_pose);

  for (tug_bresenham::LineIterator it(robot_x, robot_y, robot_end_x, robot_end_y); !it.isBeyondEnd(); ++it)
  {
    if (isCellPoseInCollision(it.getX(), it.getY(), start_pose.theta))
    {
      ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ": finish checking");
      return true;
    }
  }

  ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ": finish checking");
  return false;
}

bool CostmapCollisionCheck::isArcInCollision(
  const geometry_msgs::Pose2D& start_pose, double v, double omega, double sim_time,
  geometry_msgs::Pose2D& resulting_pose)
{
  ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ": called");

  double raw_radius = v / omega;
  double radius = std::fabs(raw_radius);

  // Compute center of rotation: center = robot_pose - radius * rot(yaw - 90deg):
  double center_x = start_pose.x - raw_radius * std::sin(start_pose.theta);
  double center_y = start_pose.y - raw_radius * -std::cos(start_pose.theta);

  double start_yaw = angles::normalize_angle(start_pose.theta);
  double delta_yaw = angles::normalize_angle(std::max(-2.0 * M_PI, std::min(omega * sim_time, 2.0 * M_PI)));
  ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ": start_yaw: " << start_yaw << ", delta_yaw: " << delta_yaw);

  double minimum_delta_yaw = angles::normalize_angle(std::sin(costmap_->getCostmap()->getResolution() / radius));

  if (std::fabs(delta_yaw) <= std::fabs(minimum_delta_yaw))
  {
    ROS_DEBUG_STREAM("check collision with pose");
    return isPoseInCollision(start_pose);
  }

  resulting_pose.theta = angles::normalize_angle(start_yaw + delta_yaw);
  resulting_pose.x = center_x + raw_radius * std::sin(resulting_pose.theta);
  resulting_pose.y = center_y + raw_radius * -std::cos(resulting_pose.theta);

  bool center_is_right = raw_radius < 0.0;

  int r = costmap_->getCostmap()->cellDistance(radius);
  int xm, ym;
  {
    std::unique_lock<costmap_2d::Costmap2D::mutex_t> costmap_lock(*this->costmap_->getCostmap()->getMutex());
    costmap_->getCostmap()->worldToMapNoBounds(center_x, center_y, xm, ym);
  }
  ROS_DEBUG_STREAM(
    __PRETTY_FUNCTION__ << ": center: " << center_x << ", " << center_y << " in map " << xm << ", " << ym);
  ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ": start checking with " << "r: " << r);

  circle_markers_.emplace_back(utils::makeCircleMarker(costmap_->getGlobalFrameID(), center_x, center_y, radius));

  for (tug_bresenham::CircleIterator it(0, 0, r); !it.isFinished(); ++it)
  {
    geometry_msgs::Pose circle_point;
    {
      std::unique_lock<costmap_2d::Costmap2D::mutex_t> costmap_lock(*this->costmap_->getCostmap()->getMutex());
      costmap_->getCostmap()->mapToWorld(
        xm + it.getX(), ym + it.getY(), circle_point.position.x, circle_point.position.y);
    }
    circle_point.orientation = tf::createQuaternionMsgFromYaw(0.);
    bresenham_circle_points_.push_back(circle_point);

    if (isInCollisionArcPoint(xm, it.getX(), ym, it.getY(), start_yaw, delta_yaw, center_is_right))
    {
      ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ": finish checking");
      return true;
    }
  }

  ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ": finish checking");
  return false;
}

bool CostmapCollisionCheck::isLineInCollision(
  const geometry_msgs::Pose2D& start_pose, double vx, double vy, double omega, double sim_time,
  geometry_msgs::Pose2D& resulting_pose)
{
  ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ": called");

  double robot_end_pose_x =
    start_pose.x + cos(start_pose.theta) * vx * sim_time - sin(start_pose.theta) * vy * sim_time;
  double robot_end_pose_y =
    start_pose.y + sin(start_pose.theta) * vx * sim_time + cos(start_pose.theta) * vy * sim_time;

  resulting_pose.x = robot_end_pose_x;
  resulting_pose.y = robot_end_pose_y;
  resulting_pose.theta = start_pose.theta;

  double start_yaw = angles::normalize_angle(start_pose.theta);
  double delta_yaw = angles::normalize_angle(std::max(-2.0 * M_PI, std::min(omega * sim_time, 2.0 * M_PI)));

  int robot_x, robot_y, robot_end_x, robot_end_y;
  {
    std::unique_lock<costmap_2d::Costmap2D::mutex_t> costmap_lock(*this->costmap_->getCostmap()->getMutex());

    costmap_->getCostmap()->worldToMapNoBounds(start_pose.x, start_pose.y, robot_x, robot_y);
    costmap_->getCostmap()->worldToMapNoBounds(robot_end_pose_x, robot_end_pose_y, robot_end_x, robot_end_y);
  }
  ROS_DEBUG_STREAM(
    __PRETTY_FUNCTION__ << ": robot_end_pose: " << robot_end_pose_x << ", " << robot_end_pose_y << " in map "
                        << robot_end_x << ", " << robot_end_y);
  ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ": start checking with robot_x: " << robot_x << " robot_y: " << robot_y
                                       << " robot_end_x: " << robot_end_x << " robot_end_y: " << robot_end_y);

  geometry_msgs::Pose tmp_robot_end_pose = utils::toPose(start_pose);
  {
    std::unique_lock<costmap_2d::Costmap2D::mutex_t> costmap_lock(*this->costmap_->getCostmap()->getMutex());

    costmap_->getCostmap()->mapToWorld(robot_end_x, robot_end_y, tmp_robot_end_pose.position.x,
                                       tmp_robot_end_pose.position.y);
  }
  path_poses_.push_back(tmp_robot_end_pose);

  const double delta_pixel_distance = std::hypot(static_cast<double>(robot_x) - static_cast<double>(robot_end_x),
                                                 static_cast<double>(robot_y) - static_cast<double>(robot_end_y));

  for (tug_bresenham::LineIterator it(robot_x, robot_y, robot_end_x, robot_end_y); !it.isBeyondEnd(); ++it)
  {
    double current_pixel_distance = std::hypot(static_cast<double>(robot_x) - static_cast<double>(it.getX()),
                                               static_cast<double>(robot_y) - static_cast<double>(it.getY()));
    double angular_to_check = angles::normalize_angle(
      current_pixel_distance / delta_pixel_distance * delta_yaw + start_yaw);
    if (isCellPoseInCollision(it.getX(), it.getY(), angular_to_check))
    {
      ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ": finish checking");
      return true;
    }
  }

  ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ": finish checking");
  return false;
}

bool CostmapCollisionCheck::isInCollisionArcPoint(
  int mx, int dx, int my, int dy, double start_yaw, double delta_yaw, bool center_is_right)
{
  ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ": called with start_yaw: " << start_yaw << ", delta_yaw: " << delta_yaw);
  double yaw = angles::normalize_angle(std::atan2(dy, dx) + (center_is_right ? -M_PI / 2.0 : M_PI / 2.0));
  ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ": got yaw: " << yaw);

  if (isYawWithinBounds(yaw, start_yaw, delta_yaw))
  {
    ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ": angular in range perform check");
    return isCellPoseInCollision(mx + dx, my + dy, yaw);
  }

  return false;
}

bool CostmapCollisionCheck::isYawWithinBounds(double yaw, double start_yaw, double delta_yaw)
{
  double current_delta = angles::shortest_angular_distance(start_yaw, yaw);
  delta_yaw = angles::normalize_angle(delta_yaw);

  if (((delta_yaw > 0.) && (current_delta < 0.)) || ((delta_yaw < 0.) && (current_delta > 0.)))
  {
    return false;
  }

  return std::fabs(current_delta) <= std::fabs(delta_yaw);
}

bool CostmapCollisionCheck::isCellPoseInCollision(int x, int y, double yaw)
{
  ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ": perform check for cell " << x << ", " << y << ", " << yaw);
  double wx, wy;
  {
    std::unique_lock<costmap_2d::Costmap2D::mutex_t> costmap_lock(*this->costmap_->getCostmap()->getMutex());
    costmap_->getCostmap()->mapToWorld(x, y, wx, wy);
  }
  return isPoseInCollision(wx, wy, yaw);
}

bool CostmapCollisionCheck::isPoseInCollision(const geometry_msgs::Pose2D& pose)
{
  return isPoseInCollision(pose.x, pose.y, pose.theta);
}

bool CostmapCollisionCheck::isPoseInCollision(double x, double y, double yaw)
{
  std::unique_lock<costmap_2d::Costmap2D::mutex_t> costmap_lock(*this->costmap_->getCostmap()->getMutex());

  ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ": called with " << x << ", " << y << ", " << yaw);

  geometry_msgs::Pose pose_to_check;
  pose_to_check.position.x = x;
  pose_to_check.position.y = y;
  pose_to_check.position.z = 0.;
  pose_to_check.orientation = tf::createQuaternionMsgFromYaw(yaw);

  const double footprint_radius = costmap_->getLayeredCostmap()->getCircumscribedRadius();
  if (((costmap_->getCostmap()->getOriginX() - footprint_radius) <= x)
      && (x <= (costmap_->getCostmap()->getOriginX() + costmap_->getCostmap()->getSizeInMetersX() + footprint_radius))
      && ((costmap_->getCostmap()->getOriginY() - footprint_radius) <= y)
      && (y <= (costmap_->getCostmap()->getOriginY() + costmap_->getCostmap()->getSizeInMetersY() + footprint_radius)))
  { // check to ensure at least one part of the robot is within the map
    ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ": get oriented footprint");
    std::vector<geometry_msgs::Point> footprint = costmap_->getRobotFootprint();

    if (padding_ > 1e-4)
    {
      ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ": padding footprint");
      costmap_2d::padFootprint(footprint, padding_);
    }

    ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ": get oriented footprint");
    // Get orienteted footprint of the robot:
    std::vector<geometry_msgs::Point> oriented_footprint;
    costmap_2d::transformFootprint(x, y, yaw, footprint, oriented_footprint);

    // Transform footprint to map cell coordinates:
    std::vector<costmap_2d::MapLocation> robot_polygon(oriented_footprint.size());
    for (size_t i = 0; i < oriented_footprint.size(); ++i)
    {
      // Using "enforce bounds" version here to make sure convex hull can be computed:
      int mx, my;
      costmap_->getCostmap()->worldToMapEnforceBounds(oriented_footprint[i].x, oriented_footprint[i].y, mx, my);
      robot_polygon[i].x = static_cast<unsigned int>(mx);
      robot_polygon[i].y = static_cast<unsigned int>(my);
      ROS_DEBUG_STREAM(
        __PRETTY_FUNCTION__ << ": transform footprint point from " << oriented_footprint[i].x << ", "
                            << oriented_footprint[i].y << " to " << robot_polygon[i].x << ", "
                            << robot_polygon[i].y);
    }

    ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ": get all cells which are in the polygon");
    // Get all cells which are in the polygon:
    std::vector<costmap_2d::MapLocation> robot_polygon_cells;
    costmap_->getCostmap()->convexFillCells(robot_polygon, robot_polygon_cells);
    ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ": got all cells which are in the polygon");

    // check for each coordinate if it is in collision
    for (const costmap_2d::MapLocation& cell : robot_polygon_cells)
    {
      const unsigned char cost = getCostSafely(cell.x, cell.y);
      ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ": got costs " << cost);
      if (cost == costmap_2d::LETHAL_OBSTACLE)
      {
        invalid_checked_poses_.push_back(pose_to_check);
        return true;
      }
    }
  }

  valid_checked_poses_.push_back(pose_to_check);
  return false;
}

unsigned char CostmapCollisionCheck::getCostSafely(unsigned int x, unsigned int y)
{
  costmap_2d::Costmap2D* costmap = costmap_->getCostmap();
  if (x < costmap->getSizeInCellsX() && y < costmap->getSizeInCellsY())
  {
    return costmap->getCost(x, y);
  }
  return costmap_2d::NO_INFORMATION;
}

bool CostmapCollisionCheck::checkCostmapForErrors()
{
  if (!costmap_)
  {
    return true;
  }

  costmap_2d::LayeredCostmap* layered_costmap = costmap_->getLayeredCostmap();

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

void CostmapCollisionCheck::publishCheckedPoses()
{
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = costmap_->getGlobalFrameID();

  if ((checked_poses_with_footprint_pub_.getNumSubscribers() > 0) && (!costmap_->getRobotFootprint().empty()))
  {
    visualization_msgs::MarkerArray checked_poses_markers;
    checked_poses_markers.markers.reserve(std::max(valid_checked_poses_.size() + invalid_checked_poses_.size(),
                                                   old_checked_poses_with_footprint_markers_));

    visualization_msgs::Marker marker;
    marker.header = header;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.ns = "checked_poses";
    marker.color.a = 1.;
    marker.scale.x = 0.1;
    marker.points = costmap_->getRobotFootprint();
    marker.points.emplace_back(marker.points.front());

    for (const auto& pose : valid_checked_poses_)
    {
      visualization_msgs::Marker new_marker = marker;
      new_marker.pose = pose;
      new_marker.color.g = 1.;
      new_marker.id = checked_poses_markers.markers.size();

      checked_poses_markers.markers.emplace_back(new_marker);
    }

    for (const auto& pose : invalid_checked_poses_)
    {
      visualization_msgs::Marker new_marker = marker;
      new_marker.pose = pose;
      new_marker.color.r = 1.;
      new_marker.id = checked_poses_markers.markers.size();

      checked_poses_markers.markers.emplace_back(new_marker);
    }

    for (size_t i = checked_poses_markers.markers.empty() ? 0 : checked_poses_markers.markers.size();
         i < old_checked_poses_with_footprint_markers_; ++i)
    {
      visualization_msgs::Marker new_marker = marker;
      new_marker.action = visualization_msgs::Marker::DELETE;
      new_marker.id = i;

      checked_poses_markers.markers.emplace_back(new_marker);
    }

    old_checked_poses_with_footprint_markers_ = checked_poses_markers.markers.size();

    checked_poses_with_footprint_pub_.publish(checked_poses_markers);
  }

  geometry_msgs::PoseArray valid_checked_poses_msg;
  valid_checked_poses_msg.header = header;
  valid_checked_poses_msg.poses.swap(valid_checked_poses_);  // Move poses into message and clear vector
  valid_checked_poses_pub_.publish(valid_checked_poses_msg);

  geometry_msgs::PoseArray invalid_checked_poses_msg;
  invalid_checked_poses_msg.header = header;
  invalid_checked_poses_msg.poses.swap(invalid_checked_poses_);  // Move poses into message and clear vector
  invalid_checked_poses_pub_.publish(invalid_checked_poses_msg);
}

void CostmapCollisionCheck::publishPathPoses()
{
  geometry_msgs::PoseArray path_poses_msg;
  path_poses_msg.header.frame_id = costmap_->getGlobalFrameID();
  path_poses_msg.header.stamp = ros::Time::now();
  path_poses_msg.poses.swap(path_poses_);  // Move poses into message and clear vector
  path_poses_pub_.publish(path_poses_msg);
}

void CostmapCollisionCheck::publishBresenhamCirclePoints()
{
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = costmap_->getGlobalFrameID();

  geometry_msgs::PoseArray bresenham_circle_points_msg;
  bresenham_circle_points_msg.header = header;
  bresenham_circle_points_msg.poses.swap(bresenham_circle_points_);  // Move poses into message and clear vector
  bresenham_circle_points_pub_.publish(bresenham_circle_points_msg);

  visualization_msgs::MarkerArray circle_markers_msg;
  circle_markers_msg.markers.swap(circle_markers_);  // Move markers into message and clear vector
  circle_markers_pub_.publish(circle_markers_msg);
}

void CostmapCollisionCheck::setPadding(double padding)
{
  padding_ = padding;
}

}
