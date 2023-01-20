#include "ros/ros.h"
#include "std_msgs/String.h"

void chatterCallback(const std_msgs::String::ConstPtr &msg) {
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  // Subscribes to topic `chatter` and calls a function each time it hear
  // something
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  // Continue to receive messages and call callbacks until node
  // is shutdown
  ros::spin();

  return 0;
}
