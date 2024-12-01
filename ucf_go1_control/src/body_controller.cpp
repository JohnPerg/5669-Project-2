#include "ucf_go1_control/body_controller.h"
#include <cmath>

using namespace ucf;

const int kLegFL = 0;
const int kLegFR = 1;
const int kLegRL = 2;
const int kLegRR = 3;

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

  footPhase_ = phaseStateMachine(currentPhase_, gait_, twist.angular);
  auto ts = ros::Time::now();
  double duration = 2.0;

  for (int legId = 0; legId < footPhase_.size(); ++legId) {
    double legPhase = footPhase_[legId];
    footPaths[legId] = swingProfile_;
    int phaseIdx = min(legPhase * swingProfile_.poses.size(), swingProfile_.poses.size() - 1);
    for (int i = 0; i < swingProfile_.poses.size(); ++i) {
      auto &pose = footPaths[legId].poses[i];
      pose.header.stamp = ts + ros::Duration(duration * i / swingProfile_.poses.size());
      pose.pose = swingProfile_.poses[phaseIdx].pose;
      // Move phase pointer, looping over the end of the vector
      phaseIdx = ++phaseIdx % swingProfile_.poses.size();
    }
  }
  // Loop overall phase to keep it between [0,1)
  currentPhase_ = fmod(currentPhase_ + (1/(duration*50)), 1.0);

  // Shift for proper frame (rel body frame)
  for (auto &pose : footPaths[kLegFL].poses) {
    pose.pose.position.x += 0.1681;
    pose.pose.position.y += -0.1300;
    // pose.pose.position.z += -0.3200;
  }
  for (auto &pose : footPaths[kLegFR].poses) {
    pose.pose.position.x += 0.1681;
    pose.pose.position.y += 0.1300;
    // pose.pose.position.z += -0.3200;
  }
  for (auto &pose : footPaths[kLegRL].poses) {
    pose.pose.position.x += -0.2081;
    pose.pose.position.y += -0.1300;
    // pose.pose.position.z += -0.3200;
  }
  for (auto &pose : footPaths[kLegRR].poses) {
    pose.pose.position.x += -0.2081;
    pose.pose.position.y += 0.1300;
    // pose.pose.position.z += -0.3200;
  }

  return footPaths;
}

double BodyController::scalePhase(double phase) {
  if (phase < 0.75) {
    double output_end = 0.50;
    double output_start = 0.0;
    double input_end = 0.75;
    double input_start = 0.0;
    double slope = 1.0 * (output_end - output_start) / (input_end - input_start);
    double contactPhase = output_start + slope * (phase - input_start);
    return contactPhase;
  } else {
    double output_end = 1.0;
    double output_start = 0.5;
    double input_end = 1.0;
    double input_start = 0.75;
    double slope = 1.0 * (output_end - output_start) / (input_end - input_start);
    double swingPhase = output_start + slope * (phase - input_start);
    return swingPhase;
  }
}

std::array<double, 4> BodyController::phaseStateMachine(double phase, BodyController::Gait gait,
                                                        geometry_msgs::Vector3 comAngle) {
  std::array<double, 4> footPhase;
  switch (gait) {
  case BodyController::Gait::kPassive: {
    footPhase[kLegFL] = 0.0;
    footPhase[kLegFR] = 0.0;
    footPhase[kLegRL] = 0.0;
    footPhase[kLegRR] = 0.0;
    // level out COM
    comAngle.x = 0.0;
    comAngle.y = 0.0;
    break;
  }
  case BodyController::Gait::kStand: {
    footPhase[kLegFL] = 0.0;
    footPhase[kLegFR] = 0.0;
    footPhase[kLegRL] = 0.0;
    footPhase[kLegRR] = 0.0;
    // level out COM
    comAngle.x = 0.0;
    comAngle.y = 0.0;
    break;
  }
  case BodyController::Gait::kWalk: {
    footPhase[0] = scalePhase(fmod(phase + 0.0, 1.0));
    footPhase[1] = scalePhase(fmod(phase + 0.5, 1.0));
    footPhase[2] = scalePhase(fmod(phase + 0.25, 1.0));
    footPhase[3] = scalePhase(fmod(phase + 0.75, 1.0));

    if (phase < 0.50) { // leans ensure stability on 3 legs
      // COM lean right
      comAngle.x = 0.50; // these angles are currently arbitrary
      comAngle.y = 0.50;
    } else if (phase >= 0.50) {
      // COM lean left
      comAngle.x = -0.50; // these angles are currently arbitrary
      comAngle.y = -0.50;
    }
    break;
  }
  case BodyController::Gait::kTrot: {
    footPhase[kLegFL] = fmod(phase + 0.5, 1.0);
    footPhase[kLegFR] = fmod(phase + 0.0, 1.0);
    footPhase[kLegRL] = fmod(phase + 0.0, 1.0);
    footPhase[kLegRR] = fmod(phase + 0.5, 1.0);
    break;
  }
  case BodyController::Gait::kCanter: {
    footPhase[kLegFL] = fmod(phase + 0.0, 1.0);
    footPhase[kLegFR] = fmod(phase + 0.3, 1.0);
    footPhase[kLegRL] = fmod(phase + 0.7, 1.0);
    footPhase[kLegRR] = fmod(phase + 0.0, 1.0);
    break;
  }
  case BodyController::Gait::kGallop: {
    footPhase[kLegFL] = fmod(phase + 0.0, 1.0);
    footPhase[kLegFR] = fmod(phase + 0.1, 1.0);
    footPhase[kLegRL] = fmod(phase + 0.6, 1.0);
    footPhase[kLegRR] = fmod(phase + 0.5, 1.0);
    break;
  }
  }
  return footPhase;
}