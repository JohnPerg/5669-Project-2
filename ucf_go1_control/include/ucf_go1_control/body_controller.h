#ifndef __BODY_CONTROLLER_H__
#define __BODY_CONTROLLER_H__

#include "geometry_msgs/Twist.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include <array>

#include "common/unitreeRobot.h"

namespace ucf {
/// @brief Provides functionality for calculating joint trajectories from desired twist
/// and current states. Provides several different gaits as options to execute.
class BodyController {
public:
  enum class Gait { kPassive = 0, kStand, kWalk, kTrot, kCanter, kGallop };

  struct PositionVelocity {
    nav_msgs::Path swingProfile;
    std::vector<geometry_msgs::Twist> velocityProfile;
  };

  BodyController(QuadrupedRobot &robotModel, Gait gait, nav_msgs::Path swingProfile,
                 std::vector<geometry_msgs::Twist> velocityProfile = {});

  /// @brief Calculate joint trajectory from current joint state and twist command
  /// @param twist Desired velocities
  /// @param jointState Current joint states
  /// @return Joint trajectories for 12 joints
  trajectory_msgs::JointTrajectory getJointTrajectory(const geometry_msgs::Twist &twist,
                                                      const sensor_msgs::JointState &jointState);

protected:
  /// @brief Get timestamped foot positions for each leg
  /// @param twist Current velocity
  /// @param jointState Current joint state
  /// @return Array of 4 leg timestamped paths
  std::array<PositionVelocity, 4> getFoot(const geometry_msgs::Twist &twist,
                                           const sensor_msgs::JointState &jointState);

  // some gaits have faster swing phases vs contact phases. This scales them
  double scalePhase(double phase);

  std::array<double, 4> phaseStateMachine(double phase, BodyController::Gait gait, geometry_msgs::Vector3 comAngle);

private:
  QuadrupedRobot &robotModel_;
  Gait gait_;
  nav_msgs::Path swingProfile_;
  std::vector<geometry_msgs::Twist> velocityProfile_;
  double currentPhase_;

  std::array<double, 4> footPhase_;
};
} // namespace ucf

#endif // __BODY_CONTROLLER_H__