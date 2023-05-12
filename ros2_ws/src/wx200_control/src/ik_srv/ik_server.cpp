#include "fk.h"
#include "ik.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "wx200_control_interfaces/srv/ik_goal.hpp"
#include <functional>
#include <rclcpp/utilities.hpp>

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class InverseKinematics : public rclcpp::Node {
public:
  InverseKinematics() : Node("motors_writer") {
    service = this->create_service<wx200_control_interfaces::srv::IKGoal>(
        "nav_to_goal",
        std::bind(&InverseKinematics::navigate_to_goal, this, _1, _2));
    publisher =
        this->create_publisher<sensor_msgs::msg::JointState>("joint_goals", 10);
    subscriber = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10,
        std::bind(&InverseKinematics::update_real_thetas, this, _1));
    tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  }

private:
  void navigate_to_goal(
      const std::shared_ptr<wx200_control_interfaces::srv::IKGoal::Request>
          request,
      std::shared_ptr<wx200_control_interfaces::srv::IKGoal::Response>
          response) {
    // Convert quaternion to euler angles
    Eigen::Quaterniond q(
        request->goal.pose.orientation.w, request->goal.pose.orientation.x,
        request->goal.pose.orientation.y, request->goal.pose.orientation.z);
    Eigen::Vector3d rpy = q.toRotationMatrix().eulerAngles(0, 1, 2);

    // End effector goal pose [x, y, z, roll, pitch]
    std::array<double, 5> goal_pose{
        request->goal.pose.position.x,
        request->goal.pose.position.y,
        request->goal.pose.position.z,
        rpy[0],
        rpy[1],
    };

    std::vector<uint8_t> motor_ids{1, 3, 4, 5, 6};
    std::array<double, 5> next_thetas;
    std::array<double, 5> current_thetas;

    bool goal_reached = false;
    while (!goal_reached) {
      // Generate new thetas and move to them
      next_thetas = IK::get_next_thetas(current_thetas, goal_pose);
      std::vector<double> next_thetas_v(next_thetas.begin(), next_thetas.end());
      sensor_msgs::msg::JointState joint_state;
      joint_state.name = {"waist_joint", "shoulder_joint", "elbow_joint",
                          "wrist_angle_joint", "wrist_roll_joint"};
      joint_state.position = next_thetas_v;
      publisher->publish(joint_state);

      // bool moving = true;
      // while (moving) {
      //   std::vector<uint32_t> moving_motors =
      //       dh.groupReadMotor(motor_ids, 122, 1);
      //   for (bool moving_motor : moving_motors) {
      //     moving &= moving_motor;
      //   }
      // }

      // Update current pose from real robot angles
      std::array<double, 5> real_thetas_array;
      std::copy_n(live_thetas.begin(), 5, real_thetas_array.begin());

      geometry_msgs::msg::TransformStamped tf;
      while (true) {
        try {
          tf = tf_buffer->lookupTransform(
              "base_link", "tool", tf2::TimePointZero);
	  break;
        } catch (tf2::TransformException &e) {
	  RCLCPP_WARN(this->get_logger(), "%s", e.what());
	  rclcpp::sleep_for(10000000ns);
	  continue;
	}
      }

      std::array<double, 3> current_xyz{
          tf.transform.translation.x,
          tf.transform.translation.y,
          tf.transform.translation.z,
      };
      // Update current pose
      std::array<double, 5> current_pose = {
          {current_xyz[0], current_xyz[1], current_xyz[2],
           real_thetas_array[4] - M_PI,
           real_thetas_array[1] + real_thetas_array[2] + real_thetas_array[3] -
               3 * M_PI}};

      // Announce current pose
      std::cout << "x: " << current_pose[0] << '\t' << "y: " << current_pose[1]
                << '\t' << "z: " << current_pose[2] << '\t'
                << "roll: " << current_pose[3] << '\t'
                << "pitch: " << current_pose[4] << '\t' << std::endl;

      // We are close to the goal if the old thetas are very close to the new
      // thetas
      bool reached = true;
      for (size_t i = 0; i < goal_pose.size(); i++) {
        if (i < 3)
          reached &= std::abs(goal_pose[i] - current_pose[i]) < 5;
        else
          reached &= std::abs(goal_pose[i] - current_pose[i]) < 0.2;
      }

      std::cout << std::endl;
      goal_reached = reached;

      // Update thetas
      current_thetas = next_thetas;
    }
    response->success = true;
  }
  void update_real_thetas(const sensor_msgs::msg::JointState msg) {
    std::copy_n(msg.position.begin(), 5, live_thetas.begin());
  }
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber;
  rclcpp::Service<wx200_control_interfaces::srv::IKGoal>::SharedPtr service;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  std::array<double, 5> live_thetas;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InverseKinematics>());

  rclcpp::shutdown();
}
