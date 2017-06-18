#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  try {
    cv::imshow("Viewer", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(30);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "CameraViewer");
  ros::NodeHandle nh;

  cv::namedWindow("Viewer");
  cv::startWindowThread();

  image_transport::ImageTransport it(nh);

  ROS_INFO("Starting camera viewer...");

  image_transport::Subscriber sub = it.subscribe("camera/rawimg", 1, imageCallback);

  ros::spin();

  ROS_INFO("Camera viewer shutting down...");

  cv::destroyWindow("Viewer");
}
