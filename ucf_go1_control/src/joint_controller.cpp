#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <ros/ros.h>
#include <udp_client_server.h>

class UDPHardwareInterface : public hardware_interface::RobotHW {
public:
    UDPHardwareInterface();
    bool init(ros::NodeHandle& nh);
    void read();
    void write();

private:
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::PositionJointInterface position_joint_interface_;

    std::vector<double> joint_positions_;
    std::vector<double> joint_velocities_;
    std::vector<double> joint_efforts_;
    std::vector<double> joint_commands_;

    // UDP communication objects
    UDPClient udp_client_;
    UDPServer udp_server_;
};

UDPHardwareInterface::UDPHardwareInterface()
    : udp_client_("robot_ip", robot_port), udp_server_(local_port) {
    joint_positions_.resize(NUM_JOINTS, 0.0);
    joint_velocities_.resize(NUM_JOINTS, 0.0);
    joint_efforts_.resize(NUM_JOINTS, 0.0);
    joint_commands_.resize(NUM_JOINTS, 0.0);
}

bool UDPHardwareInterface::init(ros::NodeHandle& nh) {
    for (size_t i = 0; i < NUM_JOINTS; ++i) {
        hardware_interface::JointStateHandle state_handle(
            "joint_" + std::to_string(i),
            &joint_positions_[i],
            &joint_velocities_[i],
            &joint_efforts_[i]);
        joint_state_interface_.registerHandle(state_handle);

        hardware_interface::JointHandle pos_handle(
            joint_state_interface_.getHandle("joint_" + std::to_string(i)),
            &joint_commands_[i]);
        position_joint_interface_.registerHandle(pos_handle);
    }

    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);
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
    udp_client_.send(
        reinterpret_cast<const char*>(joint_commands_.data()),
        joint_commands_.size() * sizeof(double));
}
