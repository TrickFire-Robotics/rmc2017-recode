#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>

#include <daybreak_2k17/PhysicalConstants.h>

#include <daybreak_2k17/BinSlideMsg.h>
#include <daybreak_2k17/MotorOutputMsg.h>

ros::Publisher motor_pub;

daybreak_2k17::MotorOutputMsg create_motor_msg(const unsigned int motorId, const float val) {
  daybreak_2k17::MotorOutputMsg generated;
  generated.header = std_msgs::Header();
  generated.motorId = motorId;
  generated.val = val;
  return generated;
}

void bin_slide_callback(const daybreak_2k17::BinSlideMsg::ConstPtr& msg) {
  ROS_INFO("Received bin slide packet: speed %f", msg->movement);
  motor_pub.publish(create_motor_msg(BIN_SLIDE, msg->movement));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Bin");
  ros::NodeHandle nh;

  motor_pub = nh.advertise<daybreak_2k17::MotorOutputMsg>("motor_output", 100);

  ROS_INFO("Bin module starting...");
  ros::Subscriber sub = nh.subscribe("bin_slide", 100, bin_slide_callback);

  ros::spin();

  ROS_INFO("Bin module shutting down...");

  return 0;
}
