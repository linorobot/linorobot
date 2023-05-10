#include "lino_pid/lino_pid_core.h"

LinoPID::LinoPID()
{
}

LinoPID::~LinoPID()
{
}

void LinoPID::publishMessage(ros::Publisher *pub_message)
{
  lino_msgs::PID msg;
  msg.p = p_;
  msg.d = d_;
  msg.i = i_;
  pub_message->publish(msg);
}

void LinoPID::messageCallback(const lino_msgs::PID::ConstPtr &msg)
{
  p_ = msg->p;
  d_ = msg->d;
  i_ = msg->i;

  //echo P,I,D
  ROS_INFO("P: %f", p_);
  ROS_INFO("D: %f", d_);
  ROS_INFO("I: %f", i_);
}

void LinoPID::configCallback(lino_pid::linoPIDConfig &config, double level)
{
  //for PID GUI
  p_ = config.p;
  d_ = config.d;
  i_ = config.i;

}
