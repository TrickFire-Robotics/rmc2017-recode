#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#include <daybreak_2k17/TankDrivePacket.h>

#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>

using namespace sf;

ros::Publisher drive_pub;

char prevKeys[255];
char currKeys[255];

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

bool keyTrig(const unsigned int key) {
  return !prevKeys[key] && currKeys[key];
}

bool keyUntrig(const unsigned int key) {
  return prevKeys[key] && !currKeys[key];
}

void keyInputsUpdate() {
  for (int i = 0; i < 255; i++) {
    prevKeys[i] = currKeys[i];
  }
  currKeys[Keyboard::W] = Keyboard::isKeyPressed(Keyboard::W);
  currKeys[Keyboard::S] = Keyboard::isKeyPressed(Keyboard::S);
  currKeys[Keyboard::A] = Keyboard::isKeyPressed(Keyboard::A);
  currKeys[Keyboard::D] = Keyboard::isKeyPressed(Keyboard::D);
}

void driveByKeyboard() {
  if (keyTrig(Keyboard::W)) {
    ROS_DEBUG("Key W pressed");
    drive_pub.publish(generateDrivePacket(1.0f, 1.0f, true, true, true, true));
  } else if (keyUntrig(Keyboard::W)) {
    ROS_DEBUG("Key W released");
    drive_pub.publish(generateDrivePacket(0.0f, 0.0f, true, true, true, true));
  }

  if (keyTrig(Keyboard::S)) {
    ROS_DEBUG("Key S pressed");
    drive_pub.publish(generateDrivePacket(-1.0f, -1.0f, true, true, true, true));
  } else if (keyUntrig(Keyboard::S)) {
    ROS_DEBUG("Key S released");
    drive_pub.publish(generateDrivePacket(0.0f, 0.0f, true, true, true, true));
  }

  if (keyTrig(Keyboard::A)) {
    ROS_DEBUG("Key A pressed");
    drive_pub.publish(generateDrivePacket(-1.0f, 1.0f, true, true, true, true));
  } else if (keyUntrig(Keyboard::A)) {
    ROS_DEBUG("Key A released");
    drive_pub.publish(generateDrivePacket(0.0f, 0.0f, true, true, true, true));
  }

  if (keyTrig(Keyboard::D)) {
    ROS_DEBUG("Key D pressed");
    drive_pub.publish(generateDrivePacket(1.0f, -1.0f, true, true, true, true));
  } else if (keyUntrig(Keyboard::D)) {
    ROS_DEBUG("Key D released");
    drive_pub.publish(generateDrivePacket(0.0f, 0.0f, true, true, true, true));
  }
}

void handleInputs() {
  keyInputsUpdate();

  driveByKeyboard();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "DriverStation");
  ros::NodeHandle nh;
  drive_pub = nh.advertise<daybreak_2k17::TankDrivePacket>("teleop_drive", 100);

  ROS_INFO("Driver station starting...");
  ROS_DEBUG("Initializing SFML window");

  RenderWindow window(sf::VideoMode(800, 600), "TrickFire Driver Station");
  window.setFramerateLimit(30);

  ROS_DEBUG("Initializing key arrays (prev/curr)");
  for (int i = 0; i < 255; i++) {
    prevKeys[i] = false;
    currKeys[i] = false;
  }

  while (window.isOpen() && ros::ok()) {
    Event event;

    while (window.pollEvent(event)) {
      if (event.type == Event::Closed) {
        window.close();
      }
    }

    handleInputs();

    window.clear(Color::Black);

    window.display();

    ros::spinOnce();
  }

  ROS_INFO("Driver station stopping...");
}
