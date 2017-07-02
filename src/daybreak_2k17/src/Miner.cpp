#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>

#include <daybreak_2k17/PhysicalConstants.h>

#include <daybreak_2k17/MinerSpinMsg.h>
#include <daybreak_2k17/MotorOutputMsg.h>

ros::Publisher motor_pub;

daybreak_2k17::MotorOutputMsg create_motor_msg(const unsigned int motorId, const float val) {
  daybreak_2k17::MotorOutputMsg generated;
  generated.header = std_msgs::Header();
  generated.motorId = motorId;
  generated.val = val;
  return generated;
}

void miner_spin_callback(const daybreak_2k17::MinerSpinMsg::ConstPtr& msg) {
  ROS_INFO("Received miner spin message: speed %f", msg->spin);
  motor_pub.publish(create_motor_msg(MINER_SPIN, msg->spin));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Miner");
  ros::NodeHandle nh;

  motor_pub = nh.advertise<daybreak_2k17::MotorOutputMsg>("motor_output", 100);

  ROS_INFO("Miner module starting...");
  ros::Subscriber miner_spin_sub = nh.subscribe("miner_spin", 100, miner_spin_callback);

  ros::spin();

  ROS_INFO("Miner module shutting down...");

  return 0;
}
