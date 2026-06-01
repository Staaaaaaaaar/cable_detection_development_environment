#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/clock.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int8.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std;

const double PI = 3.1415926;

double sensorOffsetX = 0;
double sensorOffsetY = 0;
int pubSkipNum = 1;
int pubSkipCount = 0;
bool holonomicMode = true;
double lookAheadDis = 0.3;
double posGain = 2.0;
double yawGain = 2.5;
double pathYawGain = 3.0;
bool rotateInPlaceOnPath = true;
double pathAlignYawThre = 12.0;
double maxVx = 3.0;
double minVx = 0.05;
double maxVy = 1.0;
double minVy = 0.1;
double maxYawRate = 3.0;
double minYawRate = 0.02;
double maxSpeed = 0.5;
double maxLateralSpeed = 1.0;
double maxAccel = 1.0;
double stopDisThre = 0.05;
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
float vehicleZ = 0;
float vehicleRoll = 0;
float vehiclePitch = 0;
float vehicleYaw = 0;

float vehicleXRec = 0;
float vehicleYRec = 0;
float vehicleYawRec = 0;

float cmdVx = 0;
float cmdVy = 0;
float cmdWz = 0;

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

nav_msgs::msg::Path path;
rclcpp::Node::SharedPtr nh;

float wrapAngle(float angle)
{
  while (angle > PI) angle -= 2 * PI;
  while (angle < -PI) angle += 2 * PI;
  return angle;
}

// Snap sub-deadzone commands to the SDK minimum, or zero when intentionally stopped.
void applyAxisLimit(float &v, float minAbs, float maxAbs, bool allowStop)
{
  if (allowStop && fabs(v) < 1e-4) {
    v = 0;
    return;
  }
  if (fabs(v) < 1e-4) {
    v = 0;
    return;
  }
  if (fabs(v) > maxAbs) {
    v = (v > 0 ? 1.0f : -1.0f) * maxAbs;
  } else if (fabs(v) < minAbs) {
    v = (v >= 0 ? 1.0f : -1.0f) * minAbs;
  }
}

void applyRobotCmdLimits(float &vx, float &vy, float &wz, bool atGoal)
{
  applyAxisLimit(vx, minVx, maxVx, atGoal);
  applyAxisLimit(vy, minVy, maxVy, atGoal);
  applyAxisLimit(wz, minYawRate, maxYawRate, atGoal);
}

void clampHolonomicCmd(float &vx, float &vy, float &wz, float maxLin, float maxW)
{
  if (wz > maxW) wz = maxW;
  else if (wz < -maxW) wz = -maxW;

  float linMag = sqrt(vx * vx + vy * vy);
  if (linMag > maxLin && linMag > 1e-6) {
    float scale = maxLin / linMag;
    vx *= scale;
    vy *= scale;
  }
}

void odomHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odomIn)
{
  odomTime = rclcpp::Time(odomIn->header.stamp).seconds();
  double roll, pitch, yaw;
  geometry_msgs::msg::Quaternion geoQuat = odomIn->pose.pose.orientation;
  tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  vehicleRoll = roll;
  vehiclePitch = pitch;
  vehicleYaw = yaw;
  vehicleX = odomIn->pose.pose.position.x - cos(yaw) * sensorOffsetX + sin(yaw) * sensorOffsetY;
  vehicleY = odomIn->pose.pose.position.y - sin(yaw) * sensorOffsetX - cos(yaw) * sensorOffsetY;
  vehicleZ = odomIn->pose.pose.position.z;

  if ((fabs(roll) > inclThre * PI / 180.0 || fabs(pitch) > inclThre * PI / 180.0) && useInclToStop) {
    stopInitTime = rclcpp::Time(odomIn->header.stamp).seconds();
  }

  if ((fabs(odomIn->twist.twist.angular.x) > inclRateThre * PI / 180.0 ||
       fabs(odomIn->twist.twist.angular.y) > inclRateThre * PI / 180.0) && useInclRateToSlow) {
    slowInitTime = rclcpp::Time(odomIn->header.stamp).seconds();
  }
}

void pathHandler(const nav_msgs::msg::Path::ConstSharedPtr pathIn)
{
  int pathSize = pathIn->poses.size();
  path.poses.resize(pathSize);
  for (int i = 0; i < pathSize; i++) {
    path.poses[i].pose.position.x = pathIn->poses[i].pose.position.x;
    path.poses[i].pose.position.y = pathIn->poses[i].pose.position.y;
    path.poses[i].pose.position.z = pathIn->poses[i].pose.position.z;
  }

  vehicleXRec = vehicleX;
  vehicleYRec = vehicleY;
  vehicleYawRec = vehicleYaw;

  pathPointID = 0;
  pathInit = true;
}

void goalPoseHandler(const geometry_msgs::msg::PoseStamped::ConstSharedPtr goal)
{
  goalX = goal->pose.position.x;
  goalY = goal->pose.position.y;
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf2::Quaternion(
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
  if (joySpeed > 1.0) joySpeed = 1.0;
  if (joy->axes[4] == 0) joySpeed = 0;
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
    if (joySpeed < 0) joySpeed = 0;
    else if (joySpeed > 1.0) joySpeed = 1.0;
  }
}

void stopHandler(const std_msgs::msg::Int8::ConstSharedPtr stop)
{
  safetyStop = stop->data;
}

bool computePoseServoCmd(float targetX, float targetY, float targetYaw, float speedScale)
{
  float errX = targetX - vehicleX;
  float errY = targetY - vehicleY;
  float errDist = sqrt(errX * errX + errY * errY);
  float yawErr = wrapAngle(vehicleYaw - targetYaw);

  if (errDist < goalPosThre && fabs(yawErr) < goalYawThre * PI / 180.0) {
    cmdVx = 0;
    cmdVy = 0;
    cmdWz = 0;
    return true;
  }

  float errXBody = cos(vehicleYaw) * errX + sin(vehicleYaw) * errY;
  float errYBody = -sin(vehicleYaw) * errX + cos(vehicleYaw) * errY;

  float posScale = speedScale;
  if (errDist < slowDwnDisThre) {
    posScale *= errDist / slowDwnDisThre;
  }

  cmdVx = posGain * errXBody * posScale;
  cmdVy = posGain * errYBody * posScale;

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

  clampHolonomicCmd(cmdVx, cmdVy, cmdWz, maxSpeed * speedScale, maxYawRate);
  applyRobotCmdLimits(cmdVx, cmdVy, cmdWz, false);
  return false;
}

bool computeHolonomicPathCmd(float speedScale)
{
  float vehicleXRel = cos(vehicleYawRec) * (vehicleX - vehicleXRec)
                    + sin(vehicleYawRec) * (vehicleY - vehicleYRec);
  float vehicleYRel = -sin(vehicleYawRec) * (vehicleX - vehicleXRec)
                    + cos(vehicleYawRec) * (vehicleY - vehicleYRec);

  int pathSize = path.poses.size();
  if (pathSize <= 1) return false;

  float endDisX = path.poses[pathSize - 1].pose.position.x - vehicleXRel;
  float endDisY = path.poses[pathSize - 1].pose.position.y - vehicleYRel;
  float endDis = sqrt(endDisX * endDisX + endDisY * endDisY);

  while (pathPointID < pathSize - 1) {
    float disX = path.poses[pathPointID].pose.position.x - vehicleXRel;
    float disY = path.poses[pathPointID].pose.position.y - vehicleYRel;
    float dis = sqrt(disX * disX + disY * disY);
    if (dis < lookAheadDis) pathPointID++;
    else break;
  }

  float disX = path.poses[pathPointID].pose.position.x - vehicleXRel;
  float disY = path.poses[pathPointID].pose.position.y - vehicleYRel;
  float pathDir = atan2(disY, disX);

  float deltaYaw = wrapAngle(vehicleYaw - vehicleYawRec);
  float errXBody = cos(deltaYaw) * disX + sin(deltaYaw) * disY;
  float errYBody = -sin(deltaYaw) * disX + cos(deltaYaw) * disY;

  float desiredYaw = wrapAngle(vehicleYawRec + pathDir);
  float yawErr = wrapAngle(vehicleYaw - desiredYaw);
  float yawErrAbs = fabs(yawErr);

  float posScale = speedScale;
  if (endDis < slowDwnDisThre) {
    posScale *= endDis / slowDwnDisThre;
  }

  cmdVx = posGain * errXBody * posScale;
  cmdVy = posGain * errYBody * posScale;

  float yawScale = speedScale;
  if (endDis < slowDwnDisThre) {
    yawScale *= endDis / slowDwnDisThre;
  }
  cmdWz = -pathYawGain * yawErr * yawScale;

  // During path tracking, optionally enforce "rotate first, then move".
  if (rotateInPlaceOnPath && yawErrAbs > pathAlignYawThre * PI / 180.0) {
    cmdVx = 0;
    cmdVy = 0;
  }

  clampHolonomicCmd(cmdVx, cmdVy, cmdWz, maxSpeed * speedScale, maxYawRate);
  applyRobotCmdLimits(cmdVx, cmdVy, cmdWz, false);
  return endDis > stopDisThre;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  nh = rclcpp::Node::make_shared("pathFollower");

  nh->declare_parameter<double>("sensorOffsetX", sensorOffsetX);
  nh->declare_parameter<double>("sensorOffsetY", sensorOffsetY);
  nh->declare_parameter<int>("pubSkipNum", pubSkipNum);
  nh->declare_parameter<bool>("holonomicMode", holonomicMode);
  nh->declare_parameter<double>("lookAheadDis", lookAheadDis);
  nh->declare_parameter<double>("posGain", posGain);
  nh->declare_parameter<double>("yawGain", yawGain);
  nh->declare_parameter<double>("pathYawGain", pathYawGain);
  nh->declare_parameter<bool>("rotateInPlaceOnPath", rotateInPlaceOnPath);
  nh->declare_parameter<double>("pathAlignYawThre", pathAlignYawThre);
  nh->declare_parameter<double>("maxVx", maxVx);
  nh->declare_parameter<double>("minVx", minVx);
  nh->declare_parameter<double>("maxVy", maxVy);
  nh->declare_parameter<double>("minVy", minVy);
  nh->declare_parameter<double>("maxYawRate", maxYawRate);
  nh->declare_parameter<double>("minYawRate", minYawRate);
  nh->declare_parameter<double>("maxSpeed", maxSpeed);
  nh->declare_parameter<double>("maxLateralSpeed", maxLateralSpeed);
  nh->declare_parameter<double>("maxAccel", maxAccel);
  nh->declare_parameter<double>("stopDisThre", stopDisThre);
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
  nh->get_parameter("holonomicMode", holonomicMode);
  nh->get_parameter("lookAheadDis", lookAheadDis);
  nh->get_parameter("posGain", posGain);
  nh->get_parameter("yawGain", yawGain);
  nh->get_parameter("pathYawGain", pathYawGain);
  nh->get_parameter("rotateInPlaceOnPath", rotateInPlaceOnPath);
  nh->get_parameter("pathAlignYawThre", pathAlignYawThre);
  nh->get_parameter("maxVx", maxVx);
  nh->get_parameter("minVx", minVx);
  nh->get_parameter("maxVy", maxVy);
  nh->get_parameter("minVy", minVy);
  nh->get_parameter("maxYawRate", maxYawRate);
  nh->get_parameter("minYawRate", minYawRate);
  nh->get_parameter("maxSpeed", maxSpeed);
  nh->get_parameter("maxLateralSpeed", maxLateralSpeed);
  nh->get_parameter("maxAccel", maxAccel);
  nh->get_parameter("stopDisThre", stopDisThre);
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

  auto subOdom = nh->create_subscription<nav_msgs::msg::Odometry>("/state_estimation", 5, odomHandler);
  auto subPath = nh->create_subscription<nav_msgs::msg::Path>("/path", 5, pathHandler);
  auto subGoal = nh->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 5, goalPoseHandler);
  auto subJoystick = nh->create_subscription<sensor_msgs::msg::Joy>("/joy", 5, joystickHandler);
  auto subSpeed = nh->create_subscription<std_msgs::msg::Float32>("/speed", 5, speedHandler);
  auto subStop = nh->create_subscription<std_msgs::msg::Int8>("/stop", 5, stopHandler);
  auto pubSpeed = nh->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 5);

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = "vehicle";

  if (autonomyMode) {
    joySpeed = autonomySpeed / maxSpeed;
    if (joySpeed < 0) joySpeed = 0;
    else if (joySpeed > 1.0) joySpeed = 1.0;
  }

  rclcpp::Rate rate(100);
  bool status = rclcpp::ok();
  while (status) {
    rclcpp::spin_some(nh);

    bool active = (autonomyMode && goalValid) || pathInit;
    if (active) {
      float speedScale = joySpeed;
      if (odomTime < slowInitTime + slowTime1 && slowInitTime > 0) speedScale *= slowRate1;
      else if (odomTime < slowInitTime + slowTime1 + slowTime2 && slowInitTime > 0) speedScale *= slowRate2;

      cmdVx = 0;
      cmdVy = 0;
      cmdWz = 0;

      if (autonomyMode && goalValid && holonomicMode) {
        float goalErrX = goalX - vehicleX;
        float goalErrY = goalY - vehicleY;
        float goalDist = sqrt(goalErrX * goalErrX + goalErrY * goalErrY);
        bool usePoseServo = goalDist < poseServoDisThre || !pathInit || path.poses.size() <= 1;

        if (usePoseServo) {
          computePoseServoCmd(goalX, goalY, goalYaw, speedScale);
        } else {
          computeHolonomicPathCmd(speedScale);
        }
      } else if (pathInit && holonomicMode) {
        computeHolonomicPathCmd(speedScale);
      } else if (!holonomicMode && pathInit) {
        // Legacy unicycle fallback
        float vehicleXRel = cos(vehicleYawRec) * (vehicleX - vehicleXRec)
                          + sin(vehicleYawRec) * (vehicleY - vehicleYRec);
        float vehicleYRel = -sin(vehicleYawRec) * (vehicleX - vehicleXRec)
                          + cos(vehicleYawRec) * (vehicleY - vehicleYRec);
        int pathSize = path.poses.size();
        if (pathSize > 1) {
          while (pathPointID < pathSize - 1) {
            float disX = path.poses[pathPointID].pose.position.x - vehicleXRel;
            float disY = path.poses[pathPointID].pose.position.y - vehicleYRel;
            if (sqrt(disX * disX + disY * disY) < lookAheadDis) pathPointID++;
            else break;
          }
          float disX = path.poses[pathPointID].pose.position.x - vehicleXRel;
          float disY = path.poses[pathPointID].pose.position.y - vehicleYRel;
          float pathDir = atan2(disY, disX);
          float dirDiff = wrapAngle(vehicleYaw - vehicleYawRec - pathDir);
          cmdVx = maxSpeed * speedScale * cos(dirDiff);
          cmdWz = -pathYawGain * dirDiff;
          clampHolonomicCmd(cmdVx, cmdVy, cmdWz, maxSpeed * speedScale, maxYawRate);
          applyRobotCmdLimits(cmdVx, cmdVy, cmdWz, false);
        }
      }

      if (!autonomyMode) {
        cmdVx = maxSpeed * joySpeed;
        cmdVy = 0;
        cmdWz = maxYawRate * joyYaw;
        clampHolonomicCmd(cmdVx, cmdVy, cmdWz, maxSpeed, maxYawRate);
        applyRobotCmdLimits(cmdVx, cmdVy, cmdWz, false);
      }

      if (odomTime < stopInitTime + stopTime && stopInitTime > 0) {
        cmdVx = 0;
        cmdVy = 0;
        cmdWz = 0;
      }
      if (safetyStop >= 1) { cmdVx = 0; cmdVy = 0; }
      if (safetyStop >= 2) cmdWz = 0;

      pubSkipCount--;
      if (pubSkipCount < 0) {
        cmd_vel.header.stamp = rclcpp::Time(static_cast<uint64_t>(odomTime * 1e9));
        cmd_vel.twist.linear.x = cmdVx;
        cmd_vel.twist.linear.y = cmdVy;
        cmd_vel.twist.angular.z = cmdWz;
        pubSpeed->publish(cmd_vel);
        pubSkipCount = pubSkipNum;
      }
    }

    status = rclcpp::ok();
    rate.sleep();
  }

  return 0;
}
