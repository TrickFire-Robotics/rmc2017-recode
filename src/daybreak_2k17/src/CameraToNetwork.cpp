#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

image_transport::Publisher pub;

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  cv::Mat received = cv_bridge::toCvShare(msg, "bgr8")->image;
  cv::Mat resized;
  cv::resize(received, resized, cv::Size(512, 512));

  if (!resized.empty()) {
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", resized).toImageMsg();
    pub.publish(msg);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "CameraToNetwork");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);

  ROS_INFO("Camera to network converter starting...");

  image_transport::Subscriber sub = it.subscribe("camera/rawimg", 1, imageCallback);
  pub = it.advertise("camera/netimg", 1);

  ros::spin();

  ROS_INFO("Camera to network converter stopping...");
}
