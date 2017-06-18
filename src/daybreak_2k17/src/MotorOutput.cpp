#include <ros/ros.h>

#include <std_msgs/Header.h>
#include<std_msgs/UInt8.h>
#include<std_msgs/Float32.h>

#include<daybreak_2k17/MotorOutputMsg.h>

void motor_output_callback(const daybreak_2k17::MotorOutputMsg::ConstPtr& msg) {
  ROS_INFO("Received motor output: motor %d to val %f", msg->motorId, msg->val);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "MotorOutput");
  ros::NodeHandle nh;

  ROS_INFO("Motor output module starting...");
  ros::Subscriber sub = nh.subscribe("motor_output", 100, motor_output_callback);

  ros::spin();

  return 0;
}
