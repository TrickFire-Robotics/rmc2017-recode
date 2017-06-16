#include <ros/ros.h>
#include <opencv2/highgui.hpp>

using namespace cv;

int main(int argc, char **argv)
{
  ROS_INFO("Driver station starting...");

  ros::init(argc, argv, "DriverStation");
  ros::NodeHandle nh;

  cv::namedWindow("Hey");

  int key;

  while ((key = cv::waitKey(0)) != 27) {
    ROS_INFO("Waiting... key %d", key);
  }
  ROS_INFO("Done!");
}
