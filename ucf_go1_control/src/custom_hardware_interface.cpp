#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/velocity_joint_interface.h>  // For velocity control
#include <hardware_interface/effort_joint_interface.h>   // For effort control
#include <ros/ros.h>
#include <udp_client_server.h>

class custom_hardware_interface : public hardware_interface::RobotHW {
public:
    UDPHardwareInterface();
    bool init(ros::NodeHandle& nh);
    void read();
    void write();

private:
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_; // Exposing velocity
    hardware_interface::EffortJointInterface effort_joint_interface_;     // Exposing effort

    std::vector<double> joint_positions_;
    std::vector<double> joint_velocities_;
    std::vector<double> joint_efforts_;
    std::vector<double> joint_velocity_commands_;  // For velocity commands
    std::vector<double> joint_effort_commands_;    // For effort commands

    // UDP communication objects
    UDPClient udp_client_;
    UDPServer udp_server_;
};

UDPHardwareInterface::UDPHardwareInterface()
    : udp_client_("robot_ip", robot_port), udp_server_(local_port) {
    joint_positions_.resize(NUM_JOINTS, 0.0);
    joint_velocities_.resize(NUM_JOINTS, 0.0);
    joint_efforts_.resize(NUM_JOINTS, 0.0);
    joint_velocity_commands_.resize(NUM_JOINTS, 0.0);
    joint_effort_commands_.resize(NUM_JOINTS, 0.0);
}

bool UDPHardwareInterface::init(ros::NodeHandle& nh) {
    for (size_t i = 0; i < NUM_JOINTS; ++i) {
        // Register joint state interface
        hardware_interface::JointStateHandle state_handle(
            "joint_" + std::to_string(i),
            &joint_positions_[i],
            &joint_velocities_[i],
            &joint_efforts_[i]);
        joint_state_interface_.registerHandle(state_handle);

        // Register velocity joint interface
        hardware_interface::JointHandle velocity_handle(
            joint_state_interface_.getHandle("joint_" + std::to_string(i)),
            &joint_velocity_commands_[i]);
        velocity_joint_interface_.registerHandle(velocity_handle);

        // Register effort joint interface
        hardware_interface::JointHandle effort_handle(
            joint_state_interface_.getHandle("joint_" + std::to_string(i)),
            &joint_effort_commands_[i]);
        effort_joint_interface_.registerHandle(effort_handle);
    }

    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);  // Register velocity interface
    registerInterface(&effort_joint_interface_);    // Register effort interface
    return true;
}

void UDPHardwareInterface::read() {
    char buffer[1024];
    int bytes_received = udp_server_.receive(buffer, sizeof(buffer));
    if (bytes_received > 0) {
        const double* data = reinterpret_cast<const double*>(buffer);
        for (size_t i = 0; i < NUM_JOINTS; ++i) {
            joint_positions_[i] = data[i];
            joint_velocities_[i] = data[NUM_JOINTS + i];
        }
    }
}

void UDPHardwareInterface::write() {
    // Choose either velocity or effort commands to send
    udp_client_.send(
        reinterpret_cast<const char*>(joint_velocity_commands_.data()),  // Use velocity commands
        joint_velocity_commands_.size() * sizeof(double));
}
