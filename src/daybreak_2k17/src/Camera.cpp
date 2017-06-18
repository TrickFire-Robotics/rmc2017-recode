#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Camera");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/rawimg", 1);

  ROS_INFO("Camera module starting...");

  cv::VideoCapture cap(-1);

  if (!cap.isOpened()) {
    ROS_FATAL("Couldn't open camera");
    return 1;
  }
  ROS_DEBUG("Camera opened successfully");

  cv::Mat frame;
  sensor_msgs::ImagePtr msg;

  while (ros::ok() && cap.isOpened()) {
    cap >> frame;

    if (!frame.empty()) {
      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
      pub.publish(msg);
    }
  }

  ROS_INFO("Camera module shutting down...");
}
