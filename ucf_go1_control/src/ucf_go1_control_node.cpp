
#include "common/unitreeRobot.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include <cmath>
#include <iostream>
#include <memory>
#include <ros/package.h>

#include "ucf_go1_control/body_controller.h"
#include "ucf_go1_control/csv_loader.h"
// Make a simple cycloidal profile for testing purposes
// Phase should be [0,1). A complete Cycloid profile
// and the closed base is generated over 100 total steps. 50 are in the base
// and 50 are in the curve.
nav_msgs::Path makeCycloid() {
  auto ts = ros::Time::now();
  nav_msgs::Path cycloidPath{};
  cycloidPath.header.stamp = ts;
  cycloidPath.header.frame_id = "base";

  double kHeight = 0.1;
  double kHeightOffset = -0.225;
  double kStepLength = 0.1;
  int steps = 100;
  int kSwingSteps = 25;
  for (int t = 0; t < kSwingSteps; ++t) {
    geometry_msgs::PoseStamped pose;
    float phasePI = 2 * M_PI * t / kSwingSteps;
    pose.header.frame_id = "base";
    pose.pose.position.x = -kStepLength + 2 * kStepLength * (phasePI - sin(phasePI)) / (2 * M_PI);
    pose.pose.position.z = kHeight * (1 - cos(phasePI)) / 2 + kHeightOffset;
    cycloidPath.poses.push_back(pose);
  }
  int kContactSteps = steps - kSwingSteps;
  for (int t = kSwingSteps; t < steps; ++t) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "base";
    pose.pose.position.x = -kStepLength + 2 * kStepLength * (steps - t) / kContactSteps;
    pose.pose.position.z = kHeightOffset;
    cycloidPath.poses.push_back(pose);
  }
  return cycloidPath;
}

/// @brief Make a profile from a loaded csv
/// @param parsedCsv The parsed csv file (rows, columns)
/// @return Path, VectorTwist pair
std::pair<nav_msgs::Path, std::vector<geometry_msgs::Twist>>
makeProfileFromCsv(const std::vector<std::vector<std::string>> &parsedCsv) {
  nav_msgs::Path path;
  std::vector<geometry_msgs::Twist> vel;
  path.header.frame_id = "base";
  for (const auto &line : parsedCsv) {
    double Y, Z, Vy, Vz;
    Y = std::stod(line[0]);
    Z = std::stod(line[1]);
    Vy = std::stod(line[2]);
    Vz = std::stod(line[3]);
    geometry_msgs::PoseStamped pose{};
    pose.pose.position.x = Z;
    pose.pose.position.z = Y; // Y Was up in csv
    path.poses.push_back(pose);
    geometry_msgs::Twist twist{};
    twist.linear.y = Vz;
    twist.linear.z = Vy;
    vel.push_back(twist);
  }
  return {path, vel};
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ucf_go1_control");
  ros::NodeHandle nh;

  ucf::BodyController::Gait gait;
  if (nh.hasParam("gait")) {
    int gaitParam;
    nh.getParam("gait", gaitParam);
    gait = static_cast<ucf::BodyController::Gait>(gaitParam);
  } else {
    gait = ucf::BodyController::Gait::kPassive;
  }

  std::string profileStr;
  if (nh.hasParam("profile")) {
    nh.getParam("profile", profileStr);
  }

  auto robotModel = std::make_unique<Go1Robot>();

  geometry_msgs::Twist currentTwist;
  sensor_msgs::JointState currentState;

  nav_msgs::Path swingProfile;
  std::vector<geometry_msgs::Twist> velProfile;
  if (profileStr.empty()) {
    swingProfile = makeCycloid();
  } else {
    auto csv = ucf::loadCSV(ros::package::getPath("ucf_go1_control") + "/profiles/" + profileStr);
    auto posVel = makeProfileFromCsv(csv);
    swingProfile = posVel.first;
    velProfile = posVel.second;
    for (auto &pose : swingProfile.poses) {
      pose.pose.position.z += 0.15; // Shift profile upward
      pose.pose.position.x *= 1.0;  // Scale the step size
    }
  }

  auto bodyController = std::make_unique<ucf::BodyController>(*robotModel, gait, swingProfile, velProfile);

  ros::Subscriber sub_twist =
      nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, [&currentTwist](const geometry_msgs::Twist::ConstPtr &msg) {
        ROS_INFO("Twist received x: {%f} y: {%f} yaw: {%f}", msg->linear.x, msg->linear.y, msg->angular.z);
        currentTwist = *msg;
      });

  bool receivedState = false;
  ros::Subscriber sub_joint = nh.subscribe<sensor_msgs::JointState>(
      "joint_state", 1, [&currentState, &receivedState](const sensor_msgs::JointState::ConstPtr &msg) {
        ROS_INFO_THROTTLE(1, "Joint State received.");
        receivedState = true;
        currentState = *msg;
      });

  ros::Publisher pub_traj = nh.advertise<trajectory_msgs::JointTrajectory>("command", 1);
  // Publish path for debugging purposes
  ros::Publisher pub_profile = nh.advertise<nav_msgs::Path>("path", 1, true);
  pub_profile.publish(swingProfile);

  ros::Rate rate(10);
  bool oneShot = false;
  while (ros::ok()) {
    if (gait == ucf::BodyController::Gait::kStand) {
      if (receivedState && !oneShot) {
        auto jointTraj = bodyController->getStandTrajectory(currentState);
        pub_traj.publish(jointTraj);
        oneShot = true;
      }
    } else if (gait == ucf::BodyController::Gait::kPassive) {
      if (!oneShot) {
        auto jointTraj = bodyController->getEmptyTrajectory();
        pub_traj.publish(jointTraj);
        oneShot = true;
      }
    } else {
      auto jointTraj = bodyController->getJointTrajectory(currentTwist, currentState);
      pub_traj.publish(jointTraj);
    }
    ros::spinOnce();
    rate.sleep();
  }
}