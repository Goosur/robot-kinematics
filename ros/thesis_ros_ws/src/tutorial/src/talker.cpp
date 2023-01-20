#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv) {
  // Must call init before any ros things
  ros::init(argc,    // Num args
            argv,    // Args for ros
            "talker" // Node name
  );

  // Main access point to communications with the ROS system
  // When the node handle is destructed the node shuts down
  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>(
      "chatter", // Name of topic to advertise
      1000 // Message buffer size; discards old if there are too many
  );

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok()) {
    // Message object contains data to send over topic
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    // Send the message `msg` of type `std_msgs::String` over topic `chatter`
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}
