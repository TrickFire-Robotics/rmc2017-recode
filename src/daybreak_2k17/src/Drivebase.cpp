#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#include <daybreak_2k17/TankDrivePacket.h>

void teleop_drive_callback(const daybreak_2k17::TankDrivePacket::ConstPtr& msg) {
  ROS_INFO("Received packet: L %f R %f", msg->l, msg->r);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Drivebase");
  ros::NodeHandle nh;

  ROS_INFO("Drivebase module starting...");
  ros::Subscriber sub = nh.subscribe("teleop_drive", 100, teleop_drive_callback);

  ros::spin();

  return 0;
}
