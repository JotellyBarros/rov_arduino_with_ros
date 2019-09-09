#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <iostream>
#include "std_msgs/String.h"

class TeleopTurtle {
 private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  ros::NodeHandle nh_;

  int left_axis_forward_z, left_axis_forward_x, right_axis_forward_z, right_axis_forward_x;
  int lin_boost_, ang_boost_;
  int brake_;
  double l_scale_, a_scale_;

  int prev_lboost_;
  int prev_aboost_;
  ros::Publisher vel_pub_;
  ros::Publisher vel_pub_vel_;
  ros::Subscriber joy_sub_;

 public:
  TeleopTurtle();
};

TeleopTurtle::TeleopTurtle() : left_axis_forward_z(), left_axis_forward_x(), right_axis_forward_z(), right_axis_forward_x(), 
                              lin_boost_(), ang_boost_(), brake_(), l_scale_(), a_scale_() {
  nh_.param("left_axis_forward_z", left_axis_forward_z, left_axis_forward_z);
  nh_.param("left_axis_forward_x", left_axis_forward_x, left_axis_forward_x);

  nh_.param("right_axis_forward_z", right_axis_forward_z, right_axis_forward_z);
  nh_.param("right_axis_forward_x", right_axis_forward_x, right_axis_forward_x);
  nh_.param("axis_aboost", ang_boost_, ang_boost_);
  nh_.param("button_brake", brake_, brake_);

  prev_lboost_ = 0;
  prev_aboost_ = 0;

  l_scale_ = 10;
  a_scale_ = 1;

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);

  // Publish to arduino mega
  vel_pub_vel_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 50, &TeleopTurtle::joyCallback, this);
}

void TeleopTurtle::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
  geometry_msgs::Twist twist;

  double boost = joy->axes[lin_boost_];
  if (boost != 0) {
    l_scale_ = ((boost > 0 && boost + l_scale_ > 10) || (boost < 0 && boost + l_scale_ < 1)) ? (l_scale_) : (l_scale_ + boost);
  }

  prev_lboost_ = boost;
  boost = -1 * joy->axes[ang_boost_];

  if (boost != 0) {
    a_scale_ = ((boost > 0 && boost + a_scale_ > 10) || (boost < 0 && boost + a_scale_ < 1)) ? (a_scale_) : (a_scale_ + boost);
  }

  prev_aboost_ = boost;

  /* Axes Left*/
  double left_axis_forward_x_vel = 1 * (1 - joy->axes[left_axis_forward_x]);
  twist.angular.x = (joy->buttons[brake_] == 0) ? ((l_scale_ * (left_axis_forward_x_vel - 1)) * -1) : (0);

  double left_axis_forward_z_vel = 1 * (1 - joy->axes[left_axis_forward_z]);
  twist.linear.z = (joy->buttons[brake_] == 0) ? ((l_scale_ * (left_axis_forward_z_vel - 1)) * -1) : (0);

  /* Axes Right*/
  double right_axis_forward_x_vel = 1 * (1 - joy->axes[right_axis_forward_x]);
  twist.linear.x = (joy->buttons[brake_] == 0) ? ((l_scale_ * (right_axis_forward_x_vel - 1)) * -1) : (0);

  double right_axis_forward_z_vel = 1 * (1 - joy->axes[right_axis_forward_z]);
  twist.angular.z = (joy->buttons[brake_] == 0) ? ((l_scale_ * (right_axis_forward_z_vel - 1)) * -1) : (0);

  vel_pub_.publish(twist);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "teleop_turtle");
  TeleopTurtle teleop_turtle;

  ros::spin();
}
