#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class StatePublisher : public rclcpp::Node {
public:
  StatePublisher() : Node("state_publisher") {
    this->publisher = this->create_publisher<sensor_msgs::msg::JointState>(
        "joint_states", 10);
    this->timer = this->create_wall_timer(
        500ms, std::bind(&StatePublisher::timer_callback, this));
  }

private:
  void timer_callback() {
    auto message = sensor_msgs::msg::JointState();
    message.header.stamp = this->get_clock()->now();
    publisher->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher;
};

int main(int argc, char **argv) { return 0; }