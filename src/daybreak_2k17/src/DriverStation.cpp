#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#include <vector>

#include <daybreak_2k17/TankDriveMsg.h>
#include <daybreak_2k17/BinSlideMsg.h>
#include <daybreak_2k17/BeltSpinMsg.h>

#include <SFML/System.hpp>
#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

using namespace sf;
using namespace std;

ros::Publisher drive_pub;
ros::Publisher bin_slide_pub;
ros::Publisher belt_spin_pub;

bool prevKeys[255];
bool currKeys[255];
const vector<Keyboard::Key> trackedKeys = { Keyboard::W, Keyboard::A, Keyboard::S, Keyboard::D, Keyboard::U, Keyboard::J, Keyboard::M };

double prevJoyL, prevJoyR;
double currJoyL, currJoyR;

sf::Mutex mut_camera;
cv::Mat camMat;
sf::Image camImg;
sf::Texture camTex;
sf::Sprite camSprite;

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  sf::Lock lock(mut_camera);
  cv::cvtColor(cv_bridge::toCvShare(msg, "bgr8")->image, camMat, cv::COLOR_BGR2RGBA);
  camImg.create(camMat.cols, camMat.rows, camMat.ptr());
  if (!camTex.loadFromImage(camImg)) {
    ROS_ERROR("Could not convert camera image into texture.");
  }
  camSprite.setTexture(camTex);
}

daybreak_2k17::TankDriveMsg generateDriveMsg(const float l, const float r, const bool fl, const bool rl, const bool fr, const bool rr) {
  daybreak_2k17::TankDriveMsg generated;
  generated.header = std_msgs::Header();
  generated.l = l;
  generated.r = r;
  generated.fl = fl;
  generated.rl = rl;
  generated.fr = fr;
  generated.rr = rr;
  return generated;
}

daybreak_2k17::BinSlideMsg generateBinSlideMsg(const float movement) {
  daybreak_2k17::BinSlideMsg generated;
  generated.header = std_msgs::Header();
  generated.movement = movement;
  return generated;
}

daybreak_2k17::BeltSpinMsg generateBeltSpinMsg(const float movement) {
  daybreak_2k17::BeltSpinMsg generated;
  generated.header = std_msgs::Header();
  generated.movement = movement;
  return generated;
}

bool keyTrig(const unsigned int key) {
  return !prevKeys[key] && currKeys[key];
}

bool keyUntrig(const unsigned int key) {
  return prevKeys[key] && !currKeys[key];
}

void keyInputsUpdate() {
  // Update the previous key states to match what we had last iteration
  for (int i = 0; i < 255; i++) {
    prevKeys[i] = currKeys[i];
  }
  // Update all the tracked keys
  for (vector<Keyboard::Key>::const_iterator it = trackedKeys.begin(); it != trackedKeys.end(); it++) {
    currKeys[*it] = Keyboard::isKeyPressed(*it);
  }
}

void joyInputsUpdate() {
  if (!Joystick::isConnected(0)) {
//  if (!Joystick::isConnected(0) || !Joystick::isConnected(1)) {
    ROS_DEBUG("Not enough joysticks connected, skipping joyInputsUpdate()");
    return;
  }

  prevJoyL = currJoyL;
  prevJoyR = currJoyR;

  currJoyL = Joystick::getAxisPosition(0, Joystick::Y);
//  currJoyR = Joystick::getAxisPosition(1, Joystick::Y);
  currJoyR = Joystick::getAxisPosition(0, Joystick::Y);
}

void driveByKeyboard() {
  ROS_DEBUG("Driving by keyboard");

  if (keyTrig(Keyboard::W)) {
    ROS_DEBUG("Key W pressed");
    drive_pub.publish(generateDriveMsg(1.0f, 1.0f, true, true, true, true));
  } else if (keyUntrig(Keyboard::W)) {
    ROS_DEBUG("Key W released");
    drive_pub.publish(generateDriveMsg(0.0f, 0.0f, true, true, true, true));
  }

  if (keyTrig(Keyboard::S)) {
    ROS_DEBUG("Key S pressed");
    drive_pub.publish(generateDriveMsg(-1.0f, -1.0f, true, true, true, true));
  } else if (keyUntrig(Keyboard::S)) {
    ROS_DEBUG("Key S released");
    drive_pub.publish(generateDriveMsg(0.0f, 0.0f, true, true, true, true));
  }

  if (keyTrig(Keyboard::A)) {
    ROS_DEBUG("Key A pressed");
    drive_pub.publish(generateDriveMsg(-1.0f, 1.0f, true, true, true, true));
  } else if (keyUntrig(Keyboard::A)) {
    ROS_DEBUG("Key A released");
    drive_pub.publish(generateDriveMsg(0.0f, 0.0f, true, true, true, true));
  }

  if (keyTrig(Keyboard::D)) {
    ROS_DEBUG("Key D pressed");
    drive_pub.publish(generateDriveMsg(1.0f, -1.0f, true, true, true, true));
  } else if (keyUntrig(Keyboard::D)) {
    ROS_DEBUG("Key D released");
    drive_pub.publish(generateDriveMsg(0.0f, 0.0f, true, true, true, true));
  }
}

void driveByJoystick() {
  ROS_DEBUG("Driving by joystick");

  if (abs(currJoyL - prevJoyL) > 0.05 || abs(currJoyR - prevJoyR) > 0.05) {
    ROS_DEBUG("Sending joystick drive update (deltas > threshold)");
    drive_pub.publish(generateDriveMsg(currJoyL, currJoyR, true, true, true, true));
  }
}

void operateByKeyboard() {
  ROS_DEBUG("Operating by keyboard");

  if (keyTrig(Keyboard::U)) {
    ROS_DEBUG("Key U pressed");
    bin_slide_pub.publish(generateBinSlideMsg(1.0f));
  } else if (keyUntrig(Keyboard::U)) {
    ROS_DEBUG("Key U released");
    bin_slide_pub.publish(generateBinSlideMsg(0.0f));
  }

  if (keyTrig(Keyboard::J)) {
    ROS_DEBUG("Key J pressed");
    bin_slide_pub.publish(generateBinSlideMsg(-1.0f));
  } else if (keyUntrig(Keyboard::J)) {
    ROS_DEBUG("Key J released");
    bin_slide_pub.publish(generateBinSlideMsg(0.0f));
  }

  if (keyTrig(Keyboard::M)) {
    ROS_DEBUG("Key M pressed");
    belt_spin_pub.publish(generateBeltSpinMsg(1.0f));
  } else if (keyUntrig(Keyboard::M)) {
    ROS_DEBUG("Key M released");
    belt_spin_pub.publish(generateBeltSpinMsg(0.0f));
  }
}

void handleInputs() {
  keyInputsUpdate();
  joyInputsUpdate();

  if (Joystick::isConnected(0)) {
//  if (Joystick::isConnected(0) && Joystick::isConnected(1)) {
    driveByJoystick();
  } else {
    driveByKeyboard();
    operateByKeyboard();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "DriverStation");
  ros::NodeHandle nh;
  drive_pub = nh.advertise<daybreak_2k17::TankDriveMsg>("teleop_drive", 100);
  bin_slide_pub = nh.advertise<daybreak_2k17::BinSlideMsg>("bin_slide", 100);
  belt_spin_pub = nh.advertise<daybreak_2k17::BeltSpinMsg>("belt_spin", 100);

  ROS_INFO("Driver station starting...");
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("camera/netimg", 1, imageCallback);

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

    mut_camera.lock();
    window.draw(camSprite);
    mut_camera.unlock();

    window.display();

    ros::spinOnce();
  }

  ROS_INFO("Driver station stopping...");
}
