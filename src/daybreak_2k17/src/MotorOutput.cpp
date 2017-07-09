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
bool isConnected = false;

unsigned int map_to_motor(const float val) {
  return (unsigned int)(122.85 * val + 322.125);
}

void motor_output_callback(const daybreak_2k17::MotorOutputMsg::ConstPtr& msg) {
  ROS_DEBUG("Received motor output message: motor %d to val %f", msg->motorId, msg->val);

  if (!isConnected) {
    ROS_DEBUG("Driver station not connected, ignoring motor output");
    return;
  }

#if defined(GPIO)
  pwmWrite(300 + msg->motorId, map_to_motor(msg->val));
#endif
}

bool is_driver_station_connected() {
  std::vector< std::string> nodes;
  ros::master::getNodes(nodes);
  for (std::vector<std::string>::iterator it = nodes.begin(); it != nodes.end(); it++) {
    if (*it == "/DriverStation") {
      return true;
      break;
    }
  }
  return false;
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

  // Connect-disconnect loop
  do {
    // TODO: If ROS stops this *will* crash. Doesn't affect operation much if at all, but it's not clean
    ROS_INFO("Waiting for driver station to start...");
    do {
      isConnected = is_driver_station_connected();
      if (isConnected) break;
      ros::Duration(0.5).sleep();
    } while (!isConnected);
    ROS_INFO("Driver station connected");

    while (ros::ok() && (isConnected = is_driver_station_connected())) {
      // wait until either the node stops or the driver station stops
      ros::Duration(0.5).sleep();
    }
    if (!isConnected) {
      ROS_INFO("Driver station disconnected");
    }
  } while (ros::ok());

  ROS_INFO("Motor output stopping...");

  return 0;
}
