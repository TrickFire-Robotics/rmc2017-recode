#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#include <daybreak_2k17/TankDrivePacket.h>

#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>

using namespace sf;

ros::Publisher drive_pub;

daybreak_2k17::TankDrivePacket generateDrivePacket(const float l, const float r, const bool fl, const bool rl, const bool fr, const bool rr) {
  daybreak_2k17::TankDrivePacket generated;
  generated.header = std_msgs::Header();
  generated.l = l;
  generated.r = r;
  generated.fl = fl;
  generated.rl = rl;
  generated.fr = fr;
  generated.rr = rr;
  return generated;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "DriverStation");
  ros::NodeHandle nh;
  drive_pub = nh.advertise<daybreak_2k17::TankDrivePacket>("teleop_drive", 100);

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
      drive_pub.publish(generateDrivePacket(1.0f, 1.0f, true, true, true, true));
      ros::spinOnce();
    }
    if (Keyboard::isKeyPressed(Keyboard::S)) {
      ROS_DEBUG("Key S pressed");
      drive_pub.publish(generateDrivePacket(-1.0f, -1.0f, true, true, true, true));
      ros::spinOnce();
    }
    if (Keyboard::isKeyPressed(Keyboard::A)) {
      ROS_DEBUG("Key A pressed");
      drive_pub.publish(generateDrivePacket(-1.0f, 1.0f, true, true, true, true));
      ros::spinOnce();
    }
    if (Keyboard::isKeyPressed(Keyboard::A)) {
      ROS_DEBUG("Key D pressed");
      drive_pub.publish(generateDrivePacket(1.0f, -1.0f, true, true, true, true));
      ros::spinOnce();
    }

    window.clear(Color::Black);

    window.display();
  }

  ROS_INFO("DS window closed");
}
