
#include "common/unitreeRobot.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include <cmath>
#include <iostream>
#include <memory>

#include "ucf_go1_control/body_controller.h"

// Make a simple cycloidal profile for testing purposes
// Phase should be [0,1). A complete Cycloid profile
// and the closed base is generated over 100 total steps. 50 are in the base
// and 50 are in the curve.
nav_msgs::Path makeCycloid() {
  auto ts = ros::Time::now();
  nav_msgs::Path cycloidPath{};
  cycloidPath.header.stamp = ts;
  cycloidPath.header.frame_id = "base";

  double kHeight = 0.04;
  double kHeightOffset = -0.25;
  double kStepLength = 0.1;
  int steps = 100;
  int kSwingSteps = 50;
  for (int t = 0; t < steps; ++t) {
    geometry_msgs::PoseStamped pose;
    float phasePI = 2 * M_PI * t / steps;
    pose.header.frame_id = "base";
    pose.pose.position.x = kStepLength * (phasePI - sin(phasePI)) / (2 * M_PI);
    pose.pose.position.z = kHeight * (1 - cos(phasePI)) / 2 + kHeightOffset;
    cycloidPath.poses.push_back(pose);
  }
  int kContactSteps = steps - kSwingSteps;
  for (int t = kContactSteps; t < steps; ++t) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "base";
    pose.pose.position.x = kStepLength * (steps - t) / kContactSteps;
    pose.pose.position.z = kHeightOffset;
    cycloidPath.poses.push_back(pose);
  }
  return cycloidPath;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ucf_go1_control");
  ros::NodeHandle nh;

  auto robotModel = std::make_unique<Go1Robot>();

  geometry_msgs::Twist currentTwist;
  sensor_msgs::JointState currentState;
  nav_msgs::Path swingProfile = makeCycloid();

  auto bodyController =
      std::make_unique<ucf::BodyController>(*robotModel, ucf::BodyController::Gait::kTrot, swingProfile);

  ros::Subscriber sub_twist =
      nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, [&currentTwist](const geometry_msgs::Twist::ConstPtr &msg) {
        ROS_INFO("Twist received x: {%f} y: {%f} yaw: {%f}", msg->linear.x, msg->linear.y, msg->angular.z);
        currentTwist = *msg;
      });

  ros::Subscriber sub_joint = nh.subscribe<sensor_msgs::JointState>(
      "joint_state", 1, [&currentState](const sensor_msgs::JointState::ConstPtr &msg) {
        ROS_INFO_THROTTLE(1, "Joint State received.");
        currentState = *msg;
      });

  ros::Publisher pub_traj = nh.advertise<trajectory_msgs::JointTrajectory>("command", 1);
  // Publish path for debugging purposes
  ros::Publisher pub_profile = nh.advertise<nav_msgs::Path>("path", 1, true);
  pub_profile.publish(swingProfile);

  ros::Rate rate(10);
  while (ros::ok()) {
    auto jointTraj = bodyController->getJointTrajectory(currentTwist, currentState);
    pub_traj.publish(jointTraj);
    ros::spinOnce();
    rate.sleep();
  }
}