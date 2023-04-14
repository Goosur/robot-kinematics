#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/multi_dof_joint_state.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

class StatePublisher : public rclcpp::Node {
public:
private:
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    return 0;
}
