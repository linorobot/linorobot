#ifndef SR_LINO_PID_CORE_H
#define SR_LINO_PID_CORE_H

#include "ros/ros.h"
#include "ros/time.h"

// Custom message includes. Auto-generated from msg/ directory.
#include <lino_msgs/PID.h>

// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory.
#include <lino_pid/linoPIDConfig.h>

class LinoPID
{
public:
  LinoPID();
  ~LinoPID();
  void configCallback(lino_pid::linoPIDConfig &config, double level);
  void publishMessage(ros::Publisher *pub_message);
  void messageCallback(const lino_msgs::PID::ConstPtr &msg);

  double p_;
  double d_;
  double i_;

};
#endif
