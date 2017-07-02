#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>

#include <daybreak_2k17/PhysicalConstants.h>

#include <daybreak_2k17/BinSlideMsg.h>
#include <daybreak_2k17/BeltSpinMsg.h>
#include <daybreak_2k17/MotorOutputMsg.h>

ros::Publisher motor_pub;

daybreak_2k17::MotorOutputMsg create_motor_msg(const unsigned int motorId, const float val) {
  daybreak_2k17::MotorOutputMsg generated;
  generated.motorId = motorId;
  generated.val = val;
  return generated;
}

void bin_slide_callback(const daybreak_2k17::BinSlideMsg::ConstPtr& msg) {
  ROS_INFO("Received bin slide message: speed %f", msg->movement);
  motor_pub.publish(create_motor_msg(BIN_SLIDE, msg->movement));
}

void belt_spin_callback(const daybreak_2k17::BeltSpinMsg::ConstPtr& msg) {
  ROS_INFO("Received belt spin message: speed %f", msg->movement);
  motor_pub.publish(create_motor_msg(BELT_SPIN, msg->movement));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Bin");
  ros::NodeHandle nh;

  motor_pub = nh.advertise<daybreak_2k17::MotorOutputMsg>("motor_output", 1);

  ROS_INFO("Bin module starting...");
  ros::Subscriber bin_slide_sub = nh.subscribe("bin_slide", 1, bin_slide_callback);
  ros::Subscriber belt_spin_sub = nh.subscribe("belt_spin", 1, belt_spin_callback);

  ros::spin();

  ROS_INFO("Bin module shutting down...");

  return 0;
}
