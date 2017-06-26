/*
 Copyright (c) 2016, Juan Jimeno
 All rights reserved.
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright notice,
 this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of  nor the names of its contributors may be used to
 endorse or promote products derived from this software without specific
 prior written permission.
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
 */
#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <ros.h>

//header file for publishing velocities for odom
#include <lino_msgs/Velocities.h>

//header file for cmd_subscribing to "cmd_vel"
#include <geometry_msgs/Twist.h>

//header file for pid server
#include <lino_msgs/PID.h>

//header files for imu
#include <geometry_msgs/Vector3.h>
#include <lino_msgs/Imu.h>

#include <ros/time.h>

#include <Wire.h>

#include "lino_base_config.h"
#include "Encoder.h"
#include "Motor.h"
#include "Kinematics.h"
#include "PID.h"
#include "imu.h"

#define ENCODER_OPTIMIZE_INTERRUPTS

#define IMU_PUBLISH_RATE 10 //hz
#define VEL_PUBLISH_RATE 10 //hz
#define COMMAND_RATE 10 //hz
#define DEBUG_RATE 5

#ifdef L298_DRIVER
  //left side motors
  Motor motor1(MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B); //front
  //right side motors
  Motor motor2(MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B); // front
#endif

#ifdef BTS7960_DRIVER
  //left side motors
  Motor motor1(MOTOR1_IN_A, MOTOR1_IN_B); // front
  // right side motors
  Motor motor2(MOTOR2_IN_A, MOTOR2_IN_B); // front
#endif

//COUNTS_PER_REV = 0 if no encoder
int Motor::counts_per_rev_ = COUNTS_PER_REV;

//left side encoders
Encoder motor1_encoder(MOTOR1_ENCODER_A,MOTOR1_ENCODER_B); //front

//right side encodersa
Encoder motor2_encoder(MOTOR2_ENCODER_A,MOTOR2_ENCODER_B); //front

Kinematics kinematics(MAX_RPM, WHEEL_DIAMETER, BASE_WIDTH, PWM_BITS);

PID motor1_pid(-255, 255, K_P, K_I, K_D);
PID motor2_pid(-255, 255, K_P, K_I, K_D);

double g_req_angular_vel_z = 0;
double g_req_linear_vel_x = 0;

unsigned long g_prev_command_time = 0;


bool g_is_first = true;


//callback function prototypes
void commandCallback(const geometry_msgs::Twist& cmd_msg);
void PIDCallback(const lino_msgs::PID& pid);

ros::NodeHandle nh;

ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", commandCallback);
ros::Subscriber<lino_msgs::PID> pid_sub("pid", PIDCallback);

lino_msgs::Imu raw_imu_msg;
ros::Publisher raw_imu_pub("raw_imu", &raw_imu_msg);

lino_msgs::Velocities raw_vel_msg;
ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);

void setup()
{
  nh.initNode();
  nh.getHardware()->setBaud(57600);
  nh.subscribe(pid_sub);
  nh.subscribe(cmd_sub);
  nh.advertise(raw_vel_pub);
  nh.advertise(raw_imu_pub);

  while (!nh.connected())
  {
    nh.spinOnce();
  }
  nh.loginfo("LINOBASE CONNECTED");
  delay(1);
}

void loop()
{
  static unsigned long prev_control_time = 0;
  static unsigned long publish_vel_time = 0;
  static unsigned long prev_imu_time = 0;
  static unsigned long prev_debug_time = 0;
  static bool imu_is_initialized;

  //this block drives the robot based on defined rate
  if ((millis() - prev_control_time) >= (1000 / COMMAND_RATE))
  {
    moveBase();
    prev_control_time = millis();
  }

  //this block stops the motor when no command is received
  if ((millis() - g_prev_command_time) >= 400)
  {
    stopBase();
  }

  //this block publishes velocity based on defined rate
  if ((millis() - publish_vel_time) >= (1000 / VEL_PUBLISH_RATE))
  {
    publishVelocities();
    publish_vel_time = millis();
  }

  //this block publishes the IMU data based on defined rate
  if ((millis() - prev_imu_time) >= (1000 / IMU_PUBLISH_RATE))
  {
    //sanity check if the IMU exits
    if (!imu_is_initialized)
    {
      imu_is_initialized = initIMU();
      if(imu_is_initialized)
        nh.loginfo("IMU Initialized");
      else
        nh.logfatal("IMU failed to initialize. Check your IMU connection.");
    }
    else
    {
      //publish the IMU data
      publishIMU();
    }
    prev_imu_time = millis();
  }

  //this block displays the encoder readings. change DEBUG to 0 if you don't want to display
  if(DEBUG)
  {
    if ((millis() - prev_debug_time) >= (1000 / DEBUG_RATE))
    {
      printDebug();
      prev_debug_time = millis();
    }
  }
  //call all the callbacks waiting to be called
  nh.spinOnce();
}

void PIDCallback(const lino_msgs::PID& pid)
{
  //callback function every time PID constants are received from lino_pid for tuning
  //this callback receives pid object where P,I, and D constants are stored
  motor1_pid.updateConstants(pid.p, pid.i, pid.d);
  motor2_pid.updateConstants(pid.p, pid.i, pid.d);
}

void commandCallback(const geometry_msgs::Twist& cmd_msg)
{
  //callback function every time linear and angular speed is received from 'cmd_vel' topic
  //this callback function receives cmd_msg object where linear and angular speed are stored
  g_req_linear_vel_x = cmd_msg.linear.x;
  g_req_angular_vel_z = cmd_msg.angular.z;

  g_prev_command_time = millis();
}

void moveBase()
{
  Kinematics::output req_rpm;
  //get the required rpm for each motor based on required velocities
  req_rpm = kinematics.getRPM(g_req_linear_vel_x, 0.0, g_req_angular_vel_z);

  //the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
  //the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM
  motor1.spin(motor1_pid.compute(constrain(req_rpm.motor1, -MAX_RPM, MAX_RPM), motor1.rpm));
  motor2.spin(motor2_pid.compute(constrain(req_rpm.motor2, -MAX_RPM, MAX_RPM), motor2.rpm));
}

void stopBase()
{
  g_req_linear_vel_x = 0.0;
  g_req_angular_vel_z = 0.0;
}

void publishVelocities()
{
  //update the current speed of each motor based on encoder's count
  motor1.updateSpeed(motor1_encoder.read());
  motor2.updateSpeed(motor2_encoder.read());

  Kinematics::velocities vel;
  vel = kinematics.getVelocities(motor1.rpm, motor2.rpm);

  //pass velocities to publisher object
  raw_vel_msg.linear_x = vel.linear_x;
  raw_vel_msg.linear_y = 0.0;
  raw_vel_msg.angular_z = vel.angular_z;

  //publish raw_vel_msg object to ROS
  raw_vel_pub.publish(&raw_vel_msg);
}

void publishIMU()
{
  //pass accelerometer data to imu object
  raw_imu_msg.linear_acceleration = readAccelerometer();
  
  //pass gyroscope data to imu object
  raw_imu_msg.angular_velocity = readGyroscope();

  //pass accelerometer data to imu object
  raw_imu_msg.magnetic_field = readMagnetometer();
    //publish raw_imu_msg object to ROS
  raw_imu_pub.publish(&raw_imu_msg);
}

void printDebug()
{
  char g_buffer[50];

  sprintf (g_buffer, "Encoder Left: %ld", motor1_encoder.read());
  nh.loginfo(g_buffer);
  sprintf (g_buffer, "Encoder Right: %ld", motor2_encoder.read());
  nh.loginfo(g_buffer);
}