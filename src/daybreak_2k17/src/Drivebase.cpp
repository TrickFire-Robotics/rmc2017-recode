#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>

#include <daybreak_2k17/PhysicalConstants.h>

#include <daybreak_2k17/TankDriveMsg.h>
#include <daybreak_2k17/MotorOutputMsg.h>

ros::Publisher motor_pub;

daybreak_2k17::MotorOutputMsg create_motor_msg(const unsigned int motorId, const float val) {
  daybreak_2k17::MotorOutputMsg generated;
  generated.header = std_msgs::Header();
  generated.motorId = motorId;
  generated.val = val;
  return generated;
}

void teleop_drive_callback(const daybreak_2k17::TankDriveMsg::ConstPtr& msg) {
  ROS_INFO("Received teleop drive message: L %f R %f", msg->l, msg->r);
  motor_pub.publish(create_motor_msg(DRIVE_FL, msg->fl ? msg->l : 0.0f));
  motor_pub.publish(create_motor_msg(DRIVE_RL, msg->rl ? msg->l : 0.0f));
  motor_pub.publish(create_motor_msg(DRIVE_FR, msg->fr ? msg->r : 0.0f));
  motor_pub.publish(create_motor_msg(DRIVE_RR, msg->rr ? msg->r : 0.0f));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Drivebase");
  ros::NodeHandle nh;
  motor_pub = nh.advertise<daybreak_2k17::MotorOutputMsg>("motor_output", 1);

  ROS_INFO("Drivebase module starting...");
  ros::Subscriber sub = nh.subscribe("teleop_drive", 1, teleop_drive_callback);

  ros::spin();

  ROS_INFO("Drivebase module shutting down...");

  return 0;
}
