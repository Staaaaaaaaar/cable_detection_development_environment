#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pcl/filters/voxel_grid.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2/LinearMath/Matrix3x3.h"

namespace
{
constexpr double kPi = 3.14159265358979323846;

double vehicle_length = 0.7;
double vehicle_width = 0.4;
double sensor_offset_x = 0.0;
double sensor_offset_y = 0.0;
double laser_voxel_size = 0.05;
double terrain_voxel_size = 0.2;
bool use_terrain_analysis = true;
bool check_obstacle = true;
bool check_rotation_obstacle = true;
double adjacent_range = 2.5;
double obstacle_height_threshold = 0.15;
double ground_height_threshold = 0.1;
double min_relative_z = -0.5;
double max_relative_z = 0.25;
int points_to_block_path = 2;
double safety_margin = 0.10;
double path_resolution = 0.10;
double candidate_angle_step = 5.0;
double min_candidate_length = 0.30;
double min_path_range = 0.5;
bool path_range_by_speed = true;
double goal_position_tolerance = 0.20;
double goal_yaw_tolerance = 10.0;
double goal_weight = 4.0;
double heading_weight = 0.8;
double length_weight = 1.5;
double terrain_cost_weight = 1.0;
double direction_change_weight = 0.25;
double traversal_time_weight = 0.4;
double max_forward_velocity = 3.0;
double max_lateral_velocity = 1.0;
bool autonomy_mode = true;
double max_speed = 1.0;
double autonomy_speed = 1.0;
double joy_to_speed_delay = 2.0;
double joy_to_check_obstacle_delay = 5.0;

double vehicle_x = 0.0;
double vehicle_y = 0.0;
double vehicle_z = 0.0;
double vehicle_yaw = 0.0;
double odom_time = 0.0;
double joy_time = 0.0;
double speed_scale = 1.0;
double manual_direction = 0.0;
double goal_x = 0.0;
double goal_y = 0.0;
double goal_yaw = 0.0;
bool goal_valid = false;
bool cloud_updated = false;
double previous_direction = std::numeric_limits<double>::quiet_NaN();

rclcpp::Node::SharedPtr node;
pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr local_cloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr boundary_cloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr added_obstacles(new pcl::PointCloud<pcl::PointXYZI>());
pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;

double wrapAngle(double angle)
{
  while (angle > kPi) {angle -= 2.0 * kPi;}
  while (angle < -kPi) {angle += 2.0 * kPi;}
  return angle;
}

geometry_msgs::msg::Quaternion yawQuaternion(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

void odometryHandler(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  odom_time = rclcpp::Time(msg->header.stamp).seconds();
  double roll, pitch, yaw;
  const auto & q = msg->pose.pose.orientation;
  tf2::Matrix3x3(tf2::Quaternion(q.x, q.y, q.z, q.w)).getRPY(roll, pitch, yaw);
  vehicle_yaw = yaw;
  vehicle_x = msg->pose.pose.position.x - std::cos(yaw) * sensor_offset_x +
    std::sin(yaw) * sensor_offset_y;
  vehicle_y = msg->pose.pose.position.y - std::sin(yaw) * sensor_offset_x -
    std::cos(yaw) * sensor_offset_y;
  vehicle_z = msg->pose.pose.position.z;
}

void cloudHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg, bool terrain)
{
  if (terrain != use_terrain_analysis) {return;}
  input_cloud->clear();
  pcl::fromROSMsg(*msg, *input_cloud);
  voxel_filter.setLeafSize(
    terrain ? terrain_voxel_size : laser_voxel_size,
    terrain ? terrain_voxel_size : laser_voxel_size,
    terrain ? terrain_voxel_size : laser_voxel_size);
  voxel_filter.setInputCloud(input_cloud);
  voxel_filter.filter(*filtered_cloud);
  cloud_updated = true;
}

void laserCloudHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  cloudHandler(msg, false);
}

void terrainCloudHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  cloudHandler(msg, true);
}

void goalPoseHandler(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  goal_x = msg->pose.position.x;
  goal_y = msg->pose.position.y;
  double roll, pitch, yaw;
  const auto & q = msg->pose.orientation;
  tf2::Matrix3x3(tf2::Quaternion(q.x, q.y, q.z, q.w)).getRPY(roll, pitch, yaw);
  goal_yaw = yaw;
  goal_valid = true;
  previous_direction = std::numeric_limits<double>::quiet_NaN();
}

void joystickHandler(const sensor_msgs::msg::Joy::ConstSharedPtr msg)
{
  joy_time = node->now().seconds();
  if (msg->axes.size() > 4) {
    const double forward = msg->axes[4];
    const double lateral = msg->axes[3];
    speed_scale = std::min(1.0, std::hypot(forward, lateral));
    if (speed_scale > 1e-3) {manual_direction = std::atan2(lateral, forward);}
  }
  if (msg->axes.size() > 2) {autonomy_mode = msg->axes[2] <= -0.1;}
  if (msg->axes.size() > 5) {check_obstacle = msg->axes[5] > -0.1;}
}

void speedHandler(const std_msgs::msg::Float32::ConstSharedPtr msg)
{
  if (autonomy_mode && node->now().seconds() - joy_time > joy_to_speed_delay) {
    speed_scale = std::clamp(static_cast<double>(msg->data) / max_speed, 0.0, 1.0);
  }
}

void checkObstacleHandler(const std_msgs::msg::Bool::ConstSharedPtr msg)
{
  if (autonomy_mode && node->now().seconds() - joy_time > joy_to_check_obstacle_delay) {
    check_obstacle = msg->data;
  }
}

void boundaryHandler(const geometry_msgs::msg::PolygonStamped::ConstSharedPtr msg)
{
  boundary_cloud->clear();
  if (msg->polygon.points.size() < 2) {return;}
  for (size_t i = 0; i < msg->polygon.points.size(); ++i) {
    const auto & a = msg->polygon.points[i];
    const auto & b = msg->polygon.points[(i + 1) % msg->polygon.points.size()];
    const double distance = std::hypot(b.x - a.x, b.y - a.y);
    const int samples = std::max(1, static_cast<int>(std::ceil(distance / terrain_voxel_size)));
    for (int j = 0; j <= samples; ++j) {
      pcl::PointXYZI p;
      const double ratio = static_cast<double>(j) / samples;
      p.x = a.x + ratio * (b.x - a.x);
      p.y = a.y + ratio * (b.y - a.y);
      p.z = 0.0;
      p.intensity = 100.0;
      boundary_cloud->push_back(p);
    }
  }
}

void addedObstaclesHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  added_obstacles->clear();
  pcl::fromROSMsg(*msg, *added_obstacles);
  for (auto & p : added_obstacles->points) {
    p.intensity = 200.0;
  }
}

void appendWorldPointToLocal(const pcl::PointXYZI & source, bool height_filter)
{
  const double dx = source.x - vehicle_x;
  const double dy = source.y - vehicle_y;
  pcl::PointXYZI p;
  p.x = dx * std::cos(vehicle_yaw) + dy * std::sin(vehicle_yaw);
  p.y = -dx * std::sin(vehicle_yaw) + dy * std::cos(vehicle_yaw);
  p.z = source.z - vehicle_z;
  p.intensity = source.intensity;
  if (std::hypot(p.x, p.y) > adjacent_range) {return;}
  if (height_filter && !use_terrain_analysis && (p.z <= min_relative_z || p.z >= max_relative_z)) {
    return;
  }
  if (height_filter && use_terrain_analysis && p.intensity <= ground_height_threshold) {return;}
  local_cloud->push_back(p);
}

void updateLocalCloud()
{
  local_cloud->clear();
  for (const auto & p : filtered_cloud->points) {
    appendWorldPointToLocal(p, true);
  }
  for (const auto & p : boundary_cloud->points) {
    appendWorldPointToLocal(p, false);
  }
  for (const auto & p : added_obstacles->points) {
    appendWorldPointToLocal(p, false);
  }
}

struct CollisionResult
{
  bool blocked{false};
  double terrain_cost{0.0};
};

CollisionResult checkFootprint(double center_x, double center_y, bool rotating)
{
  CollisionResult result;
  if (!check_obstacle) {return result;}
  const double half_length = vehicle_length * 0.5 + safety_margin;
  const double half_width = vehicle_width * 0.5 + safety_margin;
  const double rotation_radius = std::hypot(half_length, half_width);
  int blocking_points = 0;
  for (const auto & p : local_cloud->points) {
    const double dx = p.x - center_x;
    const double dy = p.y - center_y;
    const bool inside = rotating ? std::hypot(dx, dy) <= rotation_radius :
      (std::abs(dx) <= half_length && std::abs(dy) <= half_width);
    if (!inside) {continue;}
    const bool hard_obstacle = !use_terrain_analysis || p.intensity > obstacle_height_threshold;
    if (hard_obstacle) {
      if (++blocking_points >= points_to_block_path) {
        result.blocked = true;
        return result;
      }
    } else if (use_terrain_analysis && p.intensity > ground_height_threshold) {
      result.terrain_cost = std::max(
        result.terrain_cost,
        (p.intensity - ground_height_threshold) /
        std::max(1e-3, obstacle_height_threshold - ground_height_threshold));
    }
  }
  return result;
}

struct Candidate
{
  double direction{0.0};
  double length{0.0};
  double score{-std::numeric_limits<double>::infinity()};
  double terrain_cost{0.0};
  bool reaches_goal{false};
};

Candidate evaluateCandidate(
  double direction, double desired_direction, double requested_length,
  double relative_goal_x, double relative_goal_y, double goal_distance)
{
  Candidate candidate;
  candidate.direction = direction;
  const int steps = std::max(1, static_cast<int>(std::ceil(requested_length / path_resolution)));
  for (int i = 1; i <= steps; ++i) {
    const double distance = std::min(requested_length, i * path_resolution);
    const bool sweeps_rotation = autonomy_mode && check_rotation_obstacle &&
      std::abs(wrapAngle(goal_yaw - vehicle_yaw)) > goal_yaw_tolerance * kPi / 180.0;
    const auto collision = checkFootprint(
      distance * std::cos(direction), distance * std::sin(direction), sweeps_rotation);
    if (collision.blocked) {break;}
    candidate.length = distance;
    candidate.terrain_cost = std::max(candidate.terrain_cost, collision.terrain_cost);
  }
  if (candidate.length + 1e-3 < std::min(min_candidate_length, requested_length)) {
    return candidate;
  }

  const double end_x = candidate.length * std::cos(direction);
  const double end_y = candidate.length * std::sin(direction);
  const double remaining_goal = std::hypot(relative_goal_x - end_x, relative_goal_y - end_y);
  const double heading_error = std::abs(wrapAngle(direction - desired_direction));
  const double change_error = std::isfinite(previous_direction) ?
    std::abs(wrapAngle(direction - previous_direction)) : 0.0;
  const double direction_speed_limit = 1.0 / std::sqrt(
    std::pow(std::cos(direction) / max_forward_velocity, 2) +
    std::pow(std::sin(direction) / max_lateral_velocity, 2));
  const double usable_speed = std::max(0.05, std::min(max_speed, direction_speed_limit));
  const double traversal_time = candidate.length / usable_speed;
  candidate.reaches_goal = goal_distance <= requested_length + 1e-3 &&
    remaining_goal <= path_resolution;
  candidate.score = goal_weight * (goal_distance - remaining_goal) +
    length_weight * candidate.length - heading_weight * heading_error -
    terrain_cost_weight * candidate.terrain_cost - direction_change_weight * change_error -
    traversal_time_weight * traversal_time;
  if (candidate.reaches_goal) {candidate.score += goal_weight;}
  return candidate;
}

void publishStoppedPath(
  const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr & path_publisher,
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & candidates_publisher)
{
  nav_msgs::msg::Path path;
  path.header.stamp = rclcpp::Time(static_cast<uint64_t>(odom_time * 1e9));
  path.header.frame_id = "vehicle";
  path.poses.resize(1);
  path.poses.front().header = path.header;
  path.poses.front().pose.orientation.w = 1.0;
  path_publisher->publish(path);

  pcl::PointCloud<pcl::PointXYZI> empty;
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(empty, msg);
  msg.header = path.header;
  candidates_publisher->publish(msg);
}

void planAndPublish(
  const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr & path_publisher,
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & candidates_publisher)
{
  if (autonomy_mode && !goal_valid) {
    publishStoppedPath(path_publisher, candidates_publisher);
    return;
  }

  const double relative_goal_x = autonomy_mode ?
    (goal_x - vehicle_x) * std::cos(vehicle_yaw) + (goal_y - vehicle_y) * std::sin(vehicle_yaw) :
    adjacent_range * std::cos(manual_direction);
  const double relative_goal_y = autonomy_mode ?
    -(goal_x - vehicle_x) * std::sin(vehicle_yaw) + (goal_y - vehicle_y) * std::cos(vehicle_yaw) :
    adjacent_range * std::sin(manual_direction);
  const double goal_distance = std::hypot(relative_goal_x, relative_goal_y);
  const double desired_direction = goal_distance > 1e-6 ?
    std::atan2(relative_goal_y, relative_goal_x) : 0.0;

  if (autonomy_mode && goal_distance <= goal_position_tolerance) {
    const bool rotation_blocked = check_rotation_obstacle && checkFootprint(0.0, 0.0, true).blocked;
    if (rotation_blocked) {
      publishStoppedPath(path_publisher, candidates_publisher);
      return;
    }
    nav_msgs::msg::Path path;
    path.header.stamp = rclcpp::Time(static_cast<uint64_t>(odom_time * 1e9));
    path.header.frame_id = "vehicle";
    path.poses.resize(2);
    for (auto & pose : path.poses) {
      pose.header = path.header;
      pose.pose.position.x = relative_goal_x;
      pose.pose.position.y = relative_goal_y;
      pose.pose.orientation = yawQuaternion(wrapAngle(goal_yaw - vehicle_yaw));
    }
    path_publisher->publish(path);
    return;
  }

  double path_range = adjacent_range;
  if (path_range_by_speed) {path_range *= std::max(0.25, speed_scale);}
  path_range = std::max(min_path_range, path_range);
  // End exactly on the goal whenever it is inside the local horizon. This lets
  // the follower distinguish a collision-checked goal path from an avoidance ray.
  const double requested_length = std::min(path_range, goal_distance);

  std::vector<double> directions;
  directions.push_back(desired_direction);
  // Full-circle sampling is important for a holonomic robot escaping a local dead end.
  for (double degrees = -180.0; degrees < 180.0; degrees += candidate_angle_step) {
    directions.push_back(degrees * kPi / 180.0);
  }

  Candidate best;
  pcl::PointCloud<pcl::PointXYZI> candidate_cloud;
  for (const double direction : directions) {
    Candidate candidate = evaluateCandidate(
      direction, desired_direction, requested_length,
      relative_goal_x, relative_goal_y, goal_distance);
    if (!std::isfinite(candidate.score)) {continue;}
    for (double d = path_resolution; d <= candidate.length + 1e-3; d += path_resolution) {
      pcl::PointXYZI p;
      p.x = d * std::cos(direction);
      p.y = d * std::sin(direction);
      p.z = 0.0;
      p.intensity = candidate.reaches_goal ? 2.0 : 1.0;
      candidate_cloud.push_back(p);
    }
    if (candidate.score > best.score) {best = candidate;}
  }

  if (!std::isfinite(best.score)) {
    publishStoppedPath(path_publisher, candidates_publisher);
    return;
  }
  previous_direction = best.direction;

  nav_msgs::msg::Path path;
  path.header.stamp = rclcpp::Time(static_cast<uint64_t>(odom_time * 1e9));
  path.header.frame_id = "vehicle";
  const int point_count =
    std::max(2, static_cast<int>(std::ceil(best.length / path_resolution)) + 1);
  path.poses.resize(point_count);
  for (int i = 0; i < point_count; ++i) {
    const double distance = best.length * i / (point_count - 1);
    auto & pose = path.poses[i];
    pose.header = path.header;
    pose.pose.position.x = distance * std::cos(best.direction);
    pose.pose.position.y = distance * std::sin(best.direction);
    pose.pose.orientation = yawQuaternion(
      best.reaches_goal ?
      wrapAngle(goal_yaw - vehicle_yaw) : 0.0);
  }
  if (best.reaches_goal) {
    path.poses.back().pose.position.x = relative_goal_x;
    path.poses.back().pose.position.y = relative_goal_y;
  }
  path_publisher->publish(path);

  sensor_msgs::msg::PointCloud2 candidates_msg;
  pcl::toROSMsg(candidate_cloud, candidates_msg);
  candidates_msg.header = path.header;
  candidates_publisher->publish(candidates_msg);
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("localPlanner");

#define PARAM(name, variable) node->declare_parameter(name, variable); node->get_parameter( \
    name, \
    variable)
  PARAM("vehicleLength", vehicle_length);
  PARAM("vehicleWidth", vehicle_width);
  PARAM("sensorOffsetX", sensor_offset_x);
  PARAM("sensorOffsetY", sensor_offset_y);
  PARAM("laserVoxelSize", laser_voxel_size);
  PARAM("terrainVoxelSize", terrain_voxel_size);
  PARAM("useTerrainAnalysis", use_terrain_analysis);
  PARAM("checkObstacle", check_obstacle);
  PARAM("checkRotObstacle", check_rotation_obstacle);
  PARAM("adjacentRange", adjacent_range);
  PARAM("obstacleHeightThre", obstacle_height_threshold);
  PARAM("groundHeightThre", ground_height_threshold);
  PARAM("pointPerPathThre", points_to_block_path);
  PARAM("minRelZ", min_relative_z);
  PARAM("maxRelZ", max_relative_z);
  PARAM("safetyMargin", safety_margin);
  PARAM("pathResolution", path_resolution);
  PARAM("candidateAngleStep", candidate_angle_step);
  PARAM("minCandidateLength", min_candidate_length);
  PARAM("minPathRange", min_path_range);
  PARAM("pathRangeBySpeed", path_range_by_speed);
  PARAM("goalPositionTolerance", goal_position_tolerance);
  PARAM("goalYawTolerance", goal_yaw_tolerance);
  PARAM("goalWeight", goal_weight);
  PARAM("headingWeight", heading_weight);
  PARAM("lengthWeight", length_weight);
  PARAM("terrainCostWeight", terrain_cost_weight);
  PARAM("directionChangeWeight", direction_change_weight);
  PARAM("traversalTimeWeight", traversal_time_weight);
  PARAM("maxForwardVelocity", max_forward_velocity);
  PARAM("maxLateralVelocity", max_lateral_velocity);
  PARAM("autonomyMode", autonomy_mode);
  PARAM("maxSpeed", max_speed);
  PARAM("autonomySpeed", autonomy_speed);
  PARAM("joyToSpeedDelay", joy_to_speed_delay);
  PARAM("joyToCheckObstacleDelay", joy_to_check_obstacle_delay);
#undef PARAM

  if (max_speed <= 0 || max_forward_velocity < 0.05 || max_forward_velocity > 3.0 ||
    max_lateral_velocity < 0.1 || max_lateral_velocity > 1.0)
  {
    RCLCPP_FATAL(node->get_logger(), "Planner velocity limits violate the robot SDK ranges");
    rclcpp::shutdown();
    return 1;
  }
  speed_scale = std::clamp(autonomy_speed / max_speed, 0.0, 1.0);

  auto sub_odom = node->create_subscription<nav_msgs::msg::Odometry>(
    "/state_estimation", 5, odometryHandler);
  auto sub_scan = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/registered_scan", 5, laserCloudHandler);
  auto sub_terrain = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/terrain_map", 5, terrainCloudHandler);
  auto sub_goal = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", 5, goalPoseHandler);
  auto sub_joy = node->create_subscription<sensor_msgs::msg::Joy>(
    "/joy", 5, joystickHandler);
  auto sub_speed = node->create_subscription<std_msgs::msg::Float32>(
    "/speed", 5, speedHandler);
  auto sub_boundary = node->create_subscription<geometry_msgs::msg::PolygonStamped>(
    "/navigation_boundary", 5, boundaryHandler);
  auto sub_added = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/added_obstacles", 5, addedObstaclesHandler);
  auto sub_check = node->create_subscription<std_msgs::msg::Bool>(
    "/check_obstacle", 5, checkObstacleHandler);
  auto pub_path = node->create_publisher<nav_msgs::msg::Path>("/path", 5);
  auto pub_candidates = node->create_publisher<sensor_msgs::msg::PointCloud2>("/free_paths", 2);

  RCLCPP_INFO(
    node->get_logger(),
    "Holonomic local planner ready: %.1f deg sampling, %.2f m safety margin",
    candidate_angle_step, safety_margin);

  rclcpp::Rate rate(100);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    if (cloud_updated) {
      cloud_updated = false;
      updateLocalCloud();
      planAndPublish(pub_path, pub_candidates);
    }
    rate.sleep();
  }
  return 0;
}
