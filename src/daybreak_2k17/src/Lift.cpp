#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>

#include <daybreak_2k17/PhysicalConstants.h>

#include <daybreak_2k17/LiftMoveMsg.h>
#include <daybreak_2k17/MotorOutputMsg.h>

ros::Publisher motor_pub;

daybreak_2k17::MotorOutputMsg create_motor_msg(const unsigned int motorId, const float val) {
  daybreak_2k17::MotorOutputMsg generated;
  generated.header = std_msgs::Header();
  generated.motorId = motorId;
  generated.val = val;
  return generated;
}

void lift_move_callback(const daybreak_2k17::LiftMoveMsg::ConstPtr& msg) {
  ROS_INFO("Received lift move packet: stage %d, L %f R %f", msg->upperStage ? 2 : 1, msg->leftMove, msg->rightMove);
  motor_pub.publish(create_motor_msg(msg->upperStage ? LIFT_L2 : LIFT_L1, msg->leftMove));
  motor_pub.publish(create_motor_msg(msg->upperStage ? LIFT_R2 : LIFT_R1, msg->rightMove));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Lift");
  ros::NodeHandle nh;

  motor_pub = nh.advertise<daybreak_2k17::MotorOutputMsg>("motor_output", 100);

  ROS_INFO("Lift module starting...");
  ros::Subscriber lift_move_sub = nh.subscribe("lift_move", 100, lift_move_callback);

  ros::spin();

  ROS_INFO("Lift module shutting down...");

  return 0;
}
