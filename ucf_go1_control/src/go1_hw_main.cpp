#include <ros_control_boilerplate/generic_hw_control_loop.h>
#include <ucf_go1_control/go1_hw_interface.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "go1_hw_interface");
  ros::NodeHandle nh;

  // NOTE: We run the ROS loop in a separate thread as external calls such
  // as service callbacks to load controllers can block the (main) control loop
  ros::AsyncSpinner spinner(3);
  spinner.start();

  // Create the hardware interface specific to your robot
  std::shared_ptr<go1_control::Go1HWInterface> go1_hw_interface(new go1_control::Go1HWInterface(nh));
  go1_hw_interface->init();

  // Start the control loop
  ros_control_boilerplate::GenericHWControlLoop control_loop(nh, go1_hw_interface);
  control_loop.run();  // Blocks until shutdown signal recieved

  return 0;
}
