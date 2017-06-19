#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Time.h>

image_transport::Publisher pub;

uint32_t lastCamTime = 0;

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  if (lastCamTime == 0) {
    lastCamTime = msg->header.stamp.nsec;
  }

  cv::Mat received = cv_bridge::toCvShare(msg, "bgr8")->image;
  cv::Mat resized;
  double scale = 0.25;
  cv::resize(received, resized, cv::Size((int)(received.cols * scale), (int)(received.rows * scale)));

  if ((ros::Time::now().nsec - lastCamTime >= (0.25 * 1000000000L)) && !resized.empty()) {
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", resized).toImageMsg();
    pub.publish(msg);
    lastCamTime = ros::Time::now().nsec;
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
