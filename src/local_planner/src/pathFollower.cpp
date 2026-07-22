#include <algorithm>
#include <cmath>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int8.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

const double PI = 3.1415926;

double sensorOffsetX = 0;
double sensorOffsetY = 0;
int pubSkipNum = 1;
int pubSkipCount = 0;
double lookAheadDis = 0.3;
double posGain = 2.0;
double yawGain = 2.5;
double pathYawGain = 3.0;
double maxVx = 3.0;
double minVx = 0.05;
double maxVy = 1.0;
double minVy = 0.1;
double maxYawRate = 3.0;
double minYawRate = 0.02;
double maxSpeed = 0.5;
double maxLateralSpeed = 1.0;
double maxAccel = 1.0;
double maxYawAccel = 3.0;
double axisStopPosThre = 0.03;
double slowDwnDisThre = 1.0;
double poseServoDisThre = 1.5;
double goalPosThre = 0.05;
double goalYawThre = 5.0;
bool useInclRateToSlow = false;
double inclRateThre = 120.0;
double slowRate1 = 0.25;
double slowRate2 = 0.5;
double slowTime1 = 2.0;
double slowTime2 = 2.0;
bool useInclToStop = false;
double inclThre = 45.0;
double stopTime = 5.0;
bool autonomyMode = false;
double autonomySpeed = 0.4;
double joyToSpeedDelay = 2.0;

float joySpeed = 0;
float joySpeedRaw = 0;
float joyYaw = 0;
int safetyStop = 0;

float vehicleX = 0;
float vehicleY = 0;
float vehicleYaw = 0;

float vehicleXRec = 0;
float vehicleYRec = 0;
float vehicleYawRec = 0;

float cmdVx = 0;
float cmdVy = 0;
float cmdWz = 0;
float lastCmdVx = 0;
float lastCmdVy = 0;
float lastCmdWz = 0;
double lastCmdTime = 0;
float commandLinearLimit = 0;

float goalX = 0;
float goalY = 0;
float goalYaw = 0;
bool goalValid = false;

double odomTime = 0;
double joyTime = 0;
double slowInitTime = 0;
double stopInitTime = 0;
int pathPointID = 0;
bool pathInit = false;
bool pathEndsAtGoal = false;

nav_msgs::msg::Path path;
rclcpp::Node::SharedPtr nh;

float wrapAngle(float angle)
{
  while (angle > PI) {angle -= 2 * PI;}
  while (angle < -PI) {angle += 2 * PI;}
  return angle;
}

float clampValue(float value, float minValue, float maxValue)
{
  return std::max(minValue, std::min(value, maxValue));
}

// The robot SDK accepts exactly zero or an absolute value inside [minAbs, maxAbs].
// Rate limiting must therefore jump between zero and minAbs; values in between are illegal.
float legalRateLimitedAxis(
  float desired, float previous, float minAbs, float maxAbs,
  float maxDelta, bool forceStop)
{
  if (forceStop || fabs(desired) < 1e-6) {
    return 0;
  }

  desired = copysign(clampValue(fabs(desired), minAbs, maxAbs), desired);
  if (fabs(previous) < 1e-6) {
    return copysign(std::max(minAbs, std::min(fabs(desired), maxDelta)), desired);
  }

  // Decelerate to zero before reversing an axis; never cross the SDK dead zone.
  if (previous * desired < 0) {
    const float nextAbs = fabs(previous) - maxDelta;
    return nextAbs < minAbs ? 0 : copysign(nextAbs, previous);
  }

  const float limited = previous + clampValue(desired - previous, -maxDelta, maxDelta);
  return copysign(clampValue(fabs(limited), minAbs, maxAbs), desired);
}

void clampHolonomicCmd(float & vx, float & vy, float & wz, float maxLin, float maxW)
{
  commandLinearLimit = std::max(0.0f, maxLin);
  vx = clampValue(vx, -static_cast<float>(maxVx), static_cast<float>(maxVx));
  const float lateralLimit = std::min(maxVy, maxLateralSpeed);
  vy = clampValue(vy, -lateralLimit, lateralLimit);
  wz = clampValue(wz, -maxW, maxW);

  float linMag = sqrt(vx * vx + vy * vy);
  if (linMag > maxLin && linMag > 1e-6) {
    float scale = maxLin / linMag;
    vx *= scale;
    vy *= scale;
  }
}

void constrainRobotCommand(bool forceLinearStop, bool forceYawStop)
{
  const double now = nh->now().seconds();
  const float dt = lastCmdTime > 0 ? clampValue(now - lastCmdTime, 0.001, 0.1) : 0.02;
  const float linearDelta = std::max(0.0, maxAccel) * dt;
  const float yawDelta = std::max(0.0, maxYawAccel) * dt;

  const float desiredVx = cmdVx;
  const float desiredVy = cmdVy;
  cmdVx = legalRateLimitedAxis(
    desiredVx, lastCmdVx, minVx, maxVx, linearDelta, forceLinearStop);
  cmdVy = legalRateLimitedAxis(
    desiredVy, lastCmdVy, minVy, std::min(maxVy, maxLateralSpeed),
    linearDelta, forceLinearStop);
  cmdWz = legalRateLimitedAxis(
    cmdWz, lastCmdWz, minYawRate, maxYawRate, yawDelta, forceYawStop);

  // Minimum per-axis values can enlarge a tiny diagonal command. If that would
  // exceed the navigation speed ceiling, keep only its more significant axis.
  if (!forceLinearStop && fabs(cmdVx) > 1e-6 && fabs(cmdVy) > 1e-6 &&
    hypot(cmdVx, cmdVy) > commandLinearLimit + 1e-6)
  {
    if (fabs(desiredVx) / minVx >= fabs(desiredVy) / minVy) {
      cmdVy = 0;
    } else {
      cmdVx = 0;
    }
  }

  lastCmdVx = cmdVx;
  lastCmdVy = cmdVy;
  lastCmdWz = cmdWz;
  lastCmdTime = now;
}

void odomHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odomIn)
{
  odomTime = rclcpp::Time(odomIn->header.stamp).seconds();
  double roll, pitch, yaw;
  geometry_msgs::msg::Quaternion geoQuat = odomIn->pose.pose.orientation;
  tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(
    roll, pitch,
    yaw);

  vehicleYaw = yaw;
  vehicleX = odomIn->pose.pose.position.x - cos(yaw) * sensorOffsetX + sin(yaw) * sensorOffsetY;
  vehicleY = odomIn->pose.pose.position.y - sin(yaw) * sensorOffsetX - cos(yaw) * sensorOffsetY;
  if ((fabs(roll) > inclThre * PI / 180.0 || fabs(pitch) > inclThre * PI / 180.0) &&
    useInclToStop)
  {
    stopInitTime = rclcpp::Time(odomIn->header.stamp).seconds();
  }

  if ((fabs(odomIn->twist.twist.angular.x) > inclRateThre * PI / 180.0 ||
    fabs(odomIn->twist.twist.angular.y) > inclRateThre * PI / 180.0) && useInclRateToSlow)
  {
    slowInitTime = rclcpp::Time(odomIn->header.stamp).seconds();
  }
}

void pathHandler(const nav_msgs::msg::Path::ConstSharedPtr pathIn)
{
  int pathSize = pathIn->poses.size();
  path.poses.resize(pathSize);
  for (int i = 0; i < pathSize; i++) {
    path.poses[i] = pathIn->poses[i];
  }

  vehicleXRec = vehicleX;
  vehicleYRec = vehicleY;
  vehicleYawRec = vehicleYaw;

  pathPointID = 0;
  pathInit = true;
  pathEndsAtGoal = false;
  if (goalValid && pathSize > 1) {
    const float endX = vehicleXRec + cos(vehicleYawRec) * path.poses.back().pose.position.x -
      sin(vehicleYawRec) * path.poses.back().pose.position.y;
    const float endY = vehicleYRec + sin(vehicleYawRec) * path.poses.back().pose.position.x +
      cos(vehicleYawRec) * path.poses.back().pose.position.y;
    // The planner snaps a verified terminal path exactly onto the goal. Keep
    // this match strict so a nearby avoidance-ray endpoint cannot enable servo.
    pathEndsAtGoal = hypot(endX - goalX, endY - goalY) <= 0.02;
  }
}

void goalPoseHandler(const geometry_msgs::msg::PoseStamped::ConstSharedPtr goal)
{
  goalX = goal->pose.position.x;
  goalY = goal->pose.position.y;
  double roll, pitch, yaw;
  tf2::Matrix3x3(
    tf2::Quaternion(
      goal->pose.orientation.x, goal->pose.orientation.y,
      goal->pose.orientation.z, goal->pose.orientation.w)).getRPY(roll, pitch, yaw);
  goalYaw = yaw;
  goalValid = true;
}

void joystickHandler(const sensor_msgs::msg::Joy::ConstSharedPtr joy)
{
  joyTime = nh->now().seconds();
  joySpeedRaw = sqrt(joy->axes[3] * joy->axes[3] + joy->axes[4] * joy->axes[4]);
  joySpeed = joySpeedRaw;
  if (joySpeed > 1.0) {joySpeed = 1.0;}
  if (joy->axes[4] == 0) {joySpeed = 0;}
  joyYaw = joy->axes[3];

  if (joy->axes[2] > -0.1) {
    autonomyMode = false;
  } else {
    autonomyMode = true;
  }
}

void speedHandler(const std_msgs::msg::Float32::ConstSharedPtr speed)
{
  double speedTime = nh->now().seconds();
  if (autonomyMode && speedTime - joyTime > joyToSpeedDelay && joySpeedRaw == 0) {
    joySpeed = speed->data / maxSpeed;
    if (joySpeed < 0) {joySpeed = 0;} else if (joySpeed > 1.0) {joySpeed = 1.0;}
  }
}

void stopHandler(const std_msgs::msg::Int8::ConstSharedPtr stop)
{
  safetyStop = stop->data;
}

void computePoseServoCmd(float targetX, float targetY, float targetYaw, float speedScale)
{
  float errX = targetX - vehicleX;
  float errY = targetY - vehicleY;
  float errDist = sqrt(errX * errX + errY * errY);
  float yawErr = wrapAngle(vehicleYaw - targetYaw);

  if (errDist < goalPosThre && fabs(yawErr) < goalYawThre * PI / 180.0) {
    cmdVx = 0;
    cmdVy = 0;
    cmdWz = 0;
    return;
  }

  float errXBody = cos(vehicleYaw) * errX + sin(vehicleYaw) * errY;
  float errYBody = -sin(vehicleYaw) * errX + cos(vehicleYaw) * errY;

  float posScale = speedScale;
  if (errDist < slowDwnDisThre) {
    posScale *= errDist / slowDwnDisThre;
  }

  cmdVx = posGain * errXBody * posScale;
  cmdVy = posGain * errYBody * posScale;
  if (errDist < goalPosThre) {
    cmdVx = 0;
    cmdVy = 0;
  } else {
    if (fabs(errXBody) < axisStopPosThre) {cmdVx = 0;}
    if (fabs(errYBody) < axisStopPosThre) {cmdVy = 0;}
  }

  // Yaw control should not vanish when doing in-place rotation (errDist ~= 0).
  // Scale yaw speed by yaw error itself, not position error.
  float yawScale = speedScale;
  float yawSlowThre = 20.0 * PI / 180.0;
  float yawErrAbs = fabs(yawErr);
  if (yawErrAbs < yawSlowThre) {
    yawScale *= yawErrAbs / yawSlowThre;
  }
  if (yawScale < 0.2) {
    yawScale = 0.2;
  }
  cmdWz = -yawGain * yawErr * yawScale;
  if (fabs(yawErr) < goalYawThre * PI / 180.0) {cmdWz = 0;}

  clampHolonomicCmd(cmdVx, cmdVy, cmdWz, maxSpeed * speedScale, maxYawRate);
}

void computeHolonomicPathCmd(float speedScale)
{
  float vehicleXRel = cos(vehicleYawRec) * (vehicleX - vehicleXRec) +
    sin(vehicleYawRec) * (vehicleY - vehicleYRec);
  float vehicleYRel = -sin(vehicleYawRec) * (vehicleX - vehicleXRec) +
    cos(vehicleYawRec) * (vehicleY - vehicleYRec);

  int pathSize = path.poses.size();
  if (pathSize <= 1) {return;}

  float endDisX = path.poses[pathSize - 1].pose.position.x - vehicleXRel;
  float endDisY = path.poses[pathSize - 1].pose.position.y - vehicleYRel;
  float endDis = sqrt(endDisX * endDisX + endDisY * endDisY);

  while (pathPointID < pathSize - 1) {
    float disX = path.poses[pathPointID].pose.position.x - vehicleXRel;
    float disY = path.poses[pathPointID].pose.position.y - vehicleYRel;
    float dis = sqrt(disX * disX + disY * disY);
    if (dis < lookAheadDis) {pathPointID++;} else {break;}
  }

  float disX = path.poses[pathPointID].pose.position.x - vehicleXRel;
  float disY = path.poses[pathPointID].pose.position.y - vehicleYRel;
  float pathDir = atan2(disY, disX);

  float deltaYaw = wrapAngle(vehicleYaw - vehicleYawRec);
  float errXBody = cos(deltaYaw) * disX + sin(deltaYaw) * disY;
  float errYBody = -sin(deltaYaw) * disX + cos(deltaYaw) * disY;

  // A holonomic base can translate along the path without pointing along it.
  // In autonomy mode, control yaw independently toward the requested Goal Pose.
  float desiredYaw = (autonomyMode && goalValid) ? goalYaw : wrapAngle(vehicleYawRec + pathDir);
  float yawErr = wrapAngle(vehicleYaw - desiredYaw);

  float posScale = speedScale;
  if (endDis < slowDwnDisThre) {
    posScale *= endDis / slowDwnDisThre;
  }

  cmdVx = posGain * errXBody * posScale;
  cmdVy = posGain * errYBody * posScale;
  if (fabs(errXBody) < axisStopPosThre) {cmdVx = 0;}
  if (fabs(errYBody) < axisStopPosThre) {cmdVy = 0;}

  float yawScale = speedScale;
  if (endDis < slowDwnDisThre) {
    yawScale *= endDis / slowDwnDisThre;
  }
  cmdWz = -pathYawGain * yawErr * yawScale;
  if (fabs(yawErr) < goalYawThre * PI / 180.0) {cmdWz = 0;}

  clampHolonomicCmd(cmdVx, cmdVy, cmdWz, maxSpeed * speedScale, maxYawRate);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  nh = rclcpp::Node::make_shared("pathFollower");

  nh->declare_parameter<double>("sensorOffsetX", sensorOffsetX);
  nh->declare_parameter<double>("sensorOffsetY", sensorOffsetY);
  nh->declare_parameter<int>("pubSkipNum", pubSkipNum);
  nh->declare_parameter<double>("lookAheadDis", lookAheadDis);
  nh->declare_parameter<double>("posGain", posGain);
  nh->declare_parameter<double>("yawGain", yawGain);
  nh->declare_parameter<double>("pathYawGain", pathYawGain);
  nh->declare_parameter<double>("maxVx", maxVx);
  nh->declare_parameter<double>("minVx", minVx);
  nh->declare_parameter<double>("maxVy", maxVy);
  nh->declare_parameter<double>("minVy", minVy);
  nh->declare_parameter<double>("maxYawRate", maxYawRate);
  nh->declare_parameter<double>("minYawRate", minYawRate);
  nh->declare_parameter<double>("maxSpeed", maxSpeed);
  nh->declare_parameter<double>("maxLateralSpeed", maxLateralSpeed);
  nh->declare_parameter<double>("maxAccel", maxAccel);
  nh->declare_parameter<double>("maxYawAccel", maxYawAccel);
  nh->declare_parameter<double>("axisStopPosThre", axisStopPosThre);
  nh->declare_parameter<double>("slowDwnDisThre", slowDwnDisThre);
  nh->declare_parameter<double>("poseServoDisThre", poseServoDisThre);
  nh->declare_parameter<double>("goalPosThre", goalPosThre);
  nh->declare_parameter<double>("goalYawThre", goalYawThre);
  nh->declare_parameter<bool>("useInclRateToSlow", useInclRateToSlow);
  nh->declare_parameter<double>("inclRateThre", inclRateThre);
  nh->declare_parameter<double>("slowRate1", slowRate1);
  nh->declare_parameter<double>("slowRate2", slowRate2);
  nh->declare_parameter<double>("slowTime1", slowTime1);
  nh->declare_parameter<double>("slowTime2", slowTime2);
  nh->declare_parameter<bool>("useInclToStop", useInclToStop);
  nh->declare_parameter<double>("inclThre", inclThre);
  nh->declare_parameter<double>("stopTime", stopTime);
  nh->declare_parameter<bool>("autonomyMode", autonomyMode);
  nh->declare_parameter<double>("autonomySpeed", autonomySpeed);
  nh->declare_parameter<double>("joyToSpeedDelay", joyToSpeedDelay);

  nh->get_parameter("sensorOffsetX", sensorOffsetX);
  nh->get_parameter("sensorOffsetY", sensorOffsetY);
  nh->get_parameter("pubSkipNum", pubSkipNum);
  nh->get_parameter("lookAheadDis", lookAheadDis);
  nh->get_parameter("posGain", posGain);
  nh->get_parameter("yawGain", yawGain);
  nh->get_parameter("pathYawGain", pathYawGain);
  nh->get_parameter("maxVx", maxVx);
  nh->get_parameter("minVx", minVx);
  nh->get_parameter("maxVy", maxVy);
  nh->get_parameter("minVy", minVy);
  nh->get_parameter("maxYawRate", maxYawRate);
  nh->get_parameter("minYawRate", minYawRate);
  nh->get_parameter("maxSpeed", maxSpeed);
  nh->get_parameter("maxLateralSpeed", maxLateralSpeed);
  nh->get_parameter("maxAccel", maxAccel);
  nh->get_parameter("maxYawAccel", maxYawAccel);
  nh->get_parameter("axisStopPosThre", axisStopPosThre);
  nh->get_parameter("slowDwnDisThre", slowDwnDisThre);
  nh->get_parameter("poseServoDisThre", poseServoDisThre);
  nh->get_parameter("goalPosThre", goalPosThre);
  nh->get_parameter("goalYawThre", goalYawThre);
  nh->get_parameter("useInclRateToSlow", useInclRateToSlow);
  nh->get_parameter("inclRateThre", inclRateThre);
  nh->get_parameter("slowRate1", slowRate1);
  nh->get_parameter("slowRate2", slowRate2);
  nh->get_parameter("slowTime1", slowTime1);
  nh->get_parameter("slowTime2", slowTime2);
  nh->get_parameter("useInclToStop", useInclToStop);
  nh->get_parameter("inclThre", inclThre);
  nh->get_parameter("stopTime", stopTime);
  nh->get_parameter("autonomyMode", autonomyMode);
  nh->get_parameter("autonomySpeed", autonomySpeed);
  nh->get_parameter("joyToSpeedDelay", joyToSpeedDelay);

  const bool limitsValid = minVx >= 0.05 && maxVx <= 3.0 && minVx <= maxVx &&
    minVy >= 0.1 && maxVy <= 1.0 && minVy <= maxVy &&
    minYawRate >= 0.02 && maxYawRate <= 3.0 && minYawRate <= maxYawRate &&
    maxLateralSpeed >= minVy && maxLateralSpeed <= maxVy && maxSpeed > 0 &&
    maxAccel > 0 && maxYawAccel > 0 && axisStopPosThre >= 0;
  if (!limitsValid) {
    RCLCPP_FATAL(
      nh->get_logger(),
      "Invalid SDK limits: vx must be 0 or [0.05, 3.0], vy 0 or [0.1, 1.0], "
      "yaw_rate 0 or [0.02, 3.0]");
    rclcpp::shutdown();
    return 1;
  }

  auto subOdom = nh->create_subscription<nav_msgs::msg::Odometry>(
    "/state_estimation", 5,
    odomHandler);
  auto subPath = nh->create_subscription<nav_msgs::msg::Path>("/path", 5, pathHandler);
  auto subGoal = nh->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", 5,
    goalPoseHandler);
  auto subJoystick = nh->create_subscription<sensor_msgs::msg::Joy>("/joy", 5, joystickHandler);
  auto subSpeed = nh->create_subscription<std_msgs::msg::Float32>("/speed", 5, speedHandler);
  auto subStop = nh->create_subscription<std_msgs::msg::Int8>("/stop", 5, stopHandler);
  auto pubSpeed = nh->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 5);

  geometry_msgs::msg::Twist cmd_vel;

  if (autonomyMode) {
    joySpeed = autonomySpeed / maxSpeed;
    if (joySpeed < 0) {joySpeed = 0;} else if (joySpeed > 1.0) {joySpeed = 1.0;}
  }

  rclcpp::Rate rate(100);
  bool status = rclcpp::ok();
  while (status) {
    rclcpp::spin_some(nh);

    bool active = (autonomyMode && goalValid) || pathInit;
    if (active) {
      float speedScale = joySpeed;
      if (odomTime < slowInitTime + slowTime1 && slowInitTime > 0) {
        speedScale *= slowRate1;
      } else if (odomTime < slowInitTime + slowTime1 + slowTime2 && slowInitTime > 0) {
        speedScale *= slowRate2;
      }

      cmdVx = 0;
      cmdVy = 0;
      cmdWz = 0;

      if (autonomyMode && goalValid) {
        float goalErrX = goalX - vehicleX;
        float goalErrY = goalY - vehicleY;
        float goalDist = sqrt(goalErrX * goalErrX + goalErrY * goalErrY);
        // Direct pose servo is allowed only when the planner explicitly produced a
        // collision-free path whose endpoint is the goal. A one-point path means stop.
        bool usePoseServo = pathEndsAtGoal && pathInit && path.poses.size() > 1 &&
          goalDist < poseServoDisThre;

        if (usePoseServo) {
          computePoseServoCmd(goalX, goalY, goalYaw, speedScale);
        } else if (pathInit && path.poses.size() > 1) {
          computeHolonomicPathCmd(speedScale);
        }
      } else if (autonomyMode && pathInit) {
        computeHolonomicPathCmd(speedScale);
      }

      if (!autonomyMode) {
        cmdVx = maxSpeed * joySpeed;
        cmdVy = 0;
        cmdWz = maxYawRate * joyYaw;
        clampHolonomicCmd(cmdVx, cmdVy, cmdWz, maxSpeed, maxYawRate);
      }

      if (odomTime < stopInitTime + stopTime && stopInitTime > 0) {
        cmdVx = 0;
        cmdVy = 0;
        cmdWz = 0;
      }
      if (safetyStop >= 1) {cmdVx = 0; cmdVy = 0;}
      if (safetyStop >= 2) {cmdWz = 0;}

      pubSkipCount--;
      if (pubSkipCount < 0) {
        const bool plannerStop = autonomyMode && pathInit && path.poses.size() <= 1;
        const bool forceLinearStop = plannerStop || safetyStop >= 1 ||
          (stopInitTime > 0 && odomTime < stopInitTime + stopTime);
        const bool forceYawStop = plannerStop || safetyStop >= 2 ||
          (stopInitTime > 0 && odomTime < stopInitTime + stopTime);
        constrainRobotCommand(forceLinearStop, forceYawStop);
        cmd_vel.linear.x = cmdVx;
        cmd_vel.linear.y = cmdVy;
        cmd_vel.angular.z = cmdWz;
        pubSpeed->publish(cmd_vel);
        pubSkipCount = pubSkipNum;
      }
    }

    status = rclcpp::ok();
    rate.sleep();
  }

  return 0;
}
