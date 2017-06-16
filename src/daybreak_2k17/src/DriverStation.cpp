#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#include <daybreak_2k17/DrivePacket.h>

#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>

using namespace sf;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "DriverStation");
  ros::NodeHandle nh;

  ros::Publisher drive_pub = nh.advertise<daybreak_2k17::DrivePacket>("teleop_drive", 100);

  ROS_INFO("Driver station starting...");
  ROS_DEBUG("Initializing SFML window");

  RenderWindow window(sf::VideoMode(800, 600), "TrickFire Driver Station");

  while (window.isOpen() && ros::ok()) {
    Event event;

    while (window.pollEvent(event)) {
      if (event.type == Event::Closed) {
        window.close();
      }
    }

    if (Keyboard::isKeyPressed(Keyboard::W)) {
      ROS_DEBUG("Key W pressed");

      daybreak_2k17::DrivePacket msg;
      msg.header = std_msgs::Header();
      msg.l = 1.0f;
      msg.r = 1.0f;
      msg.fl = true;
      msg.fr = true;
      msg.rl = true;
      msg.rr = true;
      drive_pub.publish(msg);
      ros::spinOnce();
    }

    window.clear(Color::Black);

    window.display();
  }

  ROS_INFO("DS window closed");
}
