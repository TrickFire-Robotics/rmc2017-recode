#include <ros/ros.h>

#include <bondcpp/bond.h>

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

// TODO: Check how much data this uses and see if there's a better way
// TODO: A smart pointer would be much better here for null checks
bond::Bond *ds_bond = nullptr;

unsigned int map_to_motor(const float val) {
  return (unsigned int)(122.85 * val + 322.125);
}

void motor_output_callback(const daybreak_2k17::MotorOutputMsg::ConstPtr& msg) {
  ROS_DEBUG("Received motor output message: motor %d to val %f", msg->motorId, msg->val);

  if (ds_bond != nullptr && ds_bond->isBroken()) {
    ROS_DEBUG("Driver station bond broken, ignoring motor output");
    return;
  }

#if defined(GPIO)
  pwmWrite(300 + msg->motorId, map_to_motor(msg->val));
#endif
}

void bond_broken_callback() {
  ROS_INFO("Driver station bond broken, all outputs set to neutral.");

#if defined(GPIO)
  pwmWrite(316, map_to_motor(0.0f));
#endif
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "MotorOutput");
  ros::NodeHandle nh;

  ROS_INFO("Motor output starting...");
  // 10 is the number of motor outputs we have, this should be updated based on the real number if it changes
  ros::Subscriber sub = nh.subscribe("motor_output", 10, motor_output_callback);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  #if defined(GPIO)
    ROS_INFO("GPIO is enabled, setting up PCA9685 connection...");
    pwmFD = pca9685Setup(300, 0x40, 50);
    if (pwmFD < 0) {
      ROS_ERROR("Error setting up PCA9685 connection! Node will continue, but no physical outputs will be sent by this node.");
    } else {
      pca9685PWMReset(pwmFD);
      pwmWrite(316, map_to_motor(0.0f));
      ROS_INFO("PCA9685 connection established, all outputs set to neutral.");
    }
  #else
    ROS_INFO("GPIO is not enabled, no physical outputs will be sent by this node.");
  #endif

  // Bond-unbond loop
  do {
    // Set up the bond
    bond::Bond _bond("driver_station_bond", "uniqueBondId13579");
    ds_bond = &_bond;
    ds_bond->setBrokenCallback(bond_broken_callback);
    ds_bond->setHeartbeatTimeout(1.0);
    ds_bond->start();

    ROS_INFO("Waiting for driver station bond to form...");
    if (!ds_bond->waitUntilFormed(ros::Duration(-1))) {
      ROS_FATAL("Driver station bond not formed, quitting!");
      return 1;
    }
    ROS_INFO("Driver station bond formed!");
    while (ros::ok() && !ds_bond->isBroken()); // wait until either the node stops or the bond breaks

    if (!ds_bond->isBroken()) {
      // It wasn't the bond breaking, it was ROS stopping, break the bond
      ds_bond->breakBond();
    }
    // Either way, get rid of the pointer
    ds_bond = nullptr;
  } while (ros::ok());

  ROS_INFO("Motor output stopping...");

  return 0;
}
