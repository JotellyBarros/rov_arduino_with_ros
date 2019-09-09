#include <geometry_msgs/Twist.h>
#include <ros.h>

#include <Arduino.h>

ros::NodeHandle nh;

geometry_msgs::Twist twst_msg;
ros::Publisher chatter("twist", &twst_msg);
int frontL = 3;
int frontR = 9;
int rearL = 10;
int rearR = 11;
double left_percent;
double right_percent;
double left_speed;
double right_speed;

void messageMV(const geometry_msgs::Twist& move_msg) {
  tone(22, (((double)move_msg.angular.z) * 255 / 10));
}

ros::Subscriber<geometry_msgs::Twist> input("turtle1/cmd_vel", &messageMV);

void setup() {
  pinMode(22, OUTPUT);
  nh.initNode();
  nh.subscribe(input);
  nh.advertise(chatter);
}

void loop() {
  chatter.publish(&twst_msg);
  nh.spinOnce();
  delay(5);
}
