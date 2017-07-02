#include <ros/ros.h>

#include<std_msgs/Header.h>
#include<std_msgs/UInt8.h>
#include<std_msgs/Float32.h>

#include<daybreak_2k17/MotorOutputMsg.h>

#if defined(GPIO)
#include<wiringPi.h>
#include<wiringPiI2C.h>
#include<pca9685.h>
#endif

#if defined(GPIO)
int pwmFD;
#endif

unsigned int map_to_motor(const float val) {
  return (unsigned int)(122.85 * val + 322.125);
}

void motor_output_callback(const daybreak_2k17::MotorOutputMsg::ConstPtr& msg) {
  ROS_DEBUG("Received motor output message: motor %d to val %f", msg->motorId, msg->val);
#if defined(GPIO)
  pwmWrite(300 + msg->motorId, map_to_motor(msg->val));
#endif
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "MotorOutput");
  ros::NodeHandle nh;

  ROS_INFO("Motor output starting...");
  // 10 is the number of motor outputs we have, this should be updated based on the real number if it changes
  ros::Subscriber sub = nh.subscribe("motor_output", 10, motor_output_callback);

  #if defined(GPIO)
    ROS_INFO("GPIO is enabled, setting up PCA9685 connection...");
    pwmFD = pca9685Setup(300, 0x40, 50);
    if (pwmFD < 0) {
      ROS_FATAL("Error setting up PCA9685 connection! Node will continue, but no physical outputs will be sent by this node.");
    } else {
      pca9685PWMReset(pwmFD);
      pwmWrite(316, map_to_motor(0.0f));
      ROS_INFO("PCA9685 connection established, all outputs set to neutral (0.0)");
    }

  #else
    ROS_INFO("GPIO is not enabled, no physical outputs will be sent by this node.");
  #endif

  ros::spin();

  ROS_INFO("Motor output stopping...");

  return 0;
}
