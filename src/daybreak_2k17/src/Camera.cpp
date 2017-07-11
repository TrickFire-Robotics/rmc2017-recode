#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <SFML/System.hpp>

#include <daybreak_2k17/GetDownscaledImage.h>

sf::Mutex image_mutex;
cv::Mat* image;

bool downscale_image(daybreak_2k17::GetDownscaledImage::Request &req,
                     daybreak_2k17::GetDownscaledImage::Response &res) {
  sf::Lock image_lock(image_mutex);
  cv::Mat resized;
  cv::resize(*image, resized, cv::Size(0, 0), req.scale, req.scale);
  res.img = *cv_bridge::CvImage(std_msgs::Header(), "bgr8", resized).toImageMsg();

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Camera");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/rawimg", 1);
  ros::ServiceServer downscaleService = nh.advertiseService("get_downscaled_image", downscale_image);

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
    image_mutex.lock();
    image = &frame;
    image_mutex.unlock();

    if (!frame.empty()) {
      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
      pub.publish(msg);
    }
    ros::spinOnce();
  }

  ROS_INFO("Camera module stopping...");
}
