#include "ucf_go1_control/body_controller.h"
#include <cmath>

using namespace ucf;
BodyController::BodyController(QuadrupedRobot &robotModel, Gait gait, nav_msgs::Path swingProfile)
    : robotModel_(robotModel), gait_(gait), swingProfile_(swingProfile), currentPhase_(0.0) {}

trajectory_msgs::JointTrajectory BodyController::getJointTrajectory(const geometry_msgs::Twist &twist,
                                                                    const sensor_msgs::JointState &jointState) {

  trajectory_msgs::JointTrajectory trajMsg;
  trajMsg.header.frame_id = "base";
  trajMsg.header.stamp = ros::Time::now();
  trajMsg.joint_names = {"FL_hip_joint",   "FL_thigh_joint", "FL_calf_joint",  "FR_hip_joint",
                         "FR_thigh_joint", "FR_calf_joint",  "RL_hip_joint",   "RL_thigh_joint",
                         "RL_calf_joint",  "RR_hip_joint",   "RR_thigh_joint", "RR_calf_joint"};

  auto footPos = this->getFootPos(twist, jointState);
  trajMsg.points.reserve(footPos[0].poses.size());
  for (int i = 0; i < footPos[0].poses.size(); ++i) {
    Vec34 position;
    for (int j = 0; j < footPos.size(); ++j) {
      position.col(j)(0) = footPos[j].poses[i].pose.position.x;
      position.col(j)(1) = footPos[j].poses[i].pose.position.y;
      position.col(j)(2) = footPos[j].poses[i].pose.position.z;
    }
    // Do inverse kinematics over all feet positions obtaining 12 joint positions
    auto jointPositions = robotModel_.getQ(position, FrameType::BODY);
    // Drop nan-values to avoid simulation crashes. It would probably be better to
    // handle this properly so joints don't jump around if getQ fails.
    jointPositions = jointPositions.unaryExpr([](double x) { return std::isnan(x) ? 0 : x; });

    trajectory_msgs::JointTrajectoryPoint point;
    // Time_from_start is already given by the timestamped positions
    point.time_from_start = footPos[0].poses[i].header.stamp - footPos[0].poses.front().header.stamp;
    // Copy data from matrix to vector for messaging
    point.positions = std::vector<double>(jointPositions.data(),
                                          jointPositions.data() + jointPositions.rows() * jointPositions.cols());

    // TODO: Velocity FF
    trajMsg.points.push_back(point);
  }
  return trajMsg;
}

std::array<nav_msgs::Path, 4> BodyController::getFootPos(const geometry_msgs::Twist &twist,
                                                         const sensor_msgs::JointState &jointState) {

  std::array<nav_msgs::Path, 4> footPaths;

  // TODO: Determine foot path based on twist, jointState, time, and current state machine state

  auto profile = swingProfile_;
  auto ts = ros::Time::now();
  double duration = 10.0;
  int phaseIdx = currentPhase_ * profile.poses.size();
  for (int i = 0; i < profile.poses.size(); ++i) {
    auto &pose = profile.poses[i];
    pose.header.stamp = ts + ros::Duration(duration * i / profile.poses.size());
    pose.pose = swingProfile_.poses[phaseIdx].pose;
    // Move phase pointer, looping over the end of the vector
    phaseIdx = ++phaseIdx % profile.poses.size();
  }
  // Loop phase to keep it between [0,1)
  currentPhase_ = fmod(currentPhase_ + 0.01, 1.0);

  // Initial copy of standard profile
  // TODO: Get the appropriate phase and duration for each of the legs.
  footPaths[0] = profile;
  footPaths[1] = profile;
  footPaths[2] = profile;
  footPaths[3] = profile;

  // Shift for proper frame (rel body frame)
  for (auto &pose : footPaths[0].poses) {
    pose.pose.position.x += 0.1881;
    pose.pose.position.y += -0.1300;
    // pose.pose.position.z += -0.3200;
  }
  for (auto &pose : footPaths[1].poses) {
    pose.pose.position.x += 0.1881;
    pose.pose.position.y += 0.1300;
    // pose.pose.position.z += -0.3200;
  }
  for (auto &pose : footPaths[2].poses) {
    pose.pose.position.x += -0.1881;
    pose.pose.position.y += -0.1300;
    // pose.pose.position.z += -0.3200;
  }
  for (auto &pose : footPaths[3].poses) {
    pose.pose.position.x += -0.1881;
    pose.pose.position.y += 0.1300;
    // pose.pose.position.z += -0.3200;
  }

  return footPaths;
}