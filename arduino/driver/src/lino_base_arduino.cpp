
#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <ros.h>

//header file for publishing "rpm"
#include <geometry_msgs/Vector3Stamped.h>

//header file for cmd_subscribing to "cmd_vel"
#include <geometry_msgs/Twist.h>

//header file for pid server
#include <lino_pid/linoPID.h>

//header files for imu
#include <ros_arduino_msgs/RawImu.h>
#include <geometry_msgs/Vector3.h>

#include <ros/time.h>

#if defined(WIRE_T3)
#include <i2c_t3.h>
#else
#include <Wire.h>
#endif

#include "imu_configuration.h"
#include "lino_base_config.h"

#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>


#define IMU_PUBLISH_RATE 10 //hz
#define VEL_PUBLISH_RATE 10 //hz
#define COMMAND_RATE 10 //hz
#define DEBUG_RATE 5


typedef struct
{
  double previous_pid_error;
  double total_pid_error;
  long previous_encoder_ticks; //done
  double current_rpm; // done
  double required_rpm;// done
  int pwm;
}
Motor;

Motor left_motor;
Motor right_motor;

Encoder left_encoder(left_encoder_a, left_encoder_b);
Encoder right_encoder(right_encoder_a, right_encoder_b);

void drive_robot(int, int);
void check_imu();
void publish_imu();
void publish_linear_velocity(unsigned long);
void read_motor_rpm_(Motor * mot, long current_encoder_ticks, unsigned long dt );
void calculate_pwm(Motor * mot);

void command_callback( const geometry_msgs::Twist& cmd_msg);
void pid_callback( const lino_pid::linoPID& pid);

#define sign(x) (x > 0) - (x < 0)

unsigned long lastMilli = 0;       // loop timing
unsigned long lastMilliPub = 0;
unsigned long previous_command_time = 0;
unsigned long previous_control_time = 0;
unsigned long publish_vel_time = 0;
unsigned long previous_imu_time = 0;
unsigned long previous_debug_time = 0;

bool is_first = true;

float Kp = k_p;
float Kd = k_d;
float Ki = k_i;
char buffer[50];

ros::NodeHandle nh;

ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", command_callback);
ros::Subscriber<lino_pid::linoPID> pid_sub("pid", pid_callback);

ros_arduino_msgs::RawImu raw_imu_msg;
ros::Publisher raw_imu_pub("raw_imu", &raw_imu_msg);

geometry_msgs::Vector3Stamped raw_vel_msg;
ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);

void setup()
{
  pinMode(left_motor_pwm, OUTPUT);
  pinMode(left_motor_direction, OUTPUT);
  pinMode(right_motor_pwm, OUTPUT);
  pinMode(right_motor_direction, OUTPUT);
  drive_robot(0,0);

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
  nh.loginfo("Connected to microcontroller...");
  nh.loginfo("ROS Arduino IMU started.");
  
#if defined(WIRE_T3)
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_INT, I2C_RATE_400);
#else
  Wire.begin();
#endif
  delay(5);
}

void loop()
{
  if ((millis() - publish_vel_time) >= (1000 / VEL_PUBLISH_RATE))
  {
    unsigned long current_time = millis();
    publish_linear_velocity(current_time - publish_vel_time);
    publish_vel_time = millis();
  }

  if ((millis() - previous_control_time) >= (1000 / COMMAND_RATE))
  {
    unsigned long current_time = millis();
    unsigned long dt = current_time - previous_control_time;
    read_motor_rpm_(&left_motor, left_encoder.read(), dt);
    read_motor_rpm_(&right_motor, right_encoder.read(), dt);
    calculate_pwm(&left_motor);
    calculate_pwm(&right_motor);    
    drive_robot(left_motor.pwm, right_motor.pwm);
    previous_control_time = millis();
  }

  if ((millis() - previous_command_time) >= 400)
  {
    left_motor.required_rpm = 0;
    right_motor.required_rpm = 0;
    right_motor.pwm = 0;
    left_motor.pwm = 0;
    drive_robot(0, 0);
  }

  if ((millis() - previous_imu_time) >= (1000 / IMU_PUBLISH_RATE))
  {
    if (is_first)
    {
      check_imu();
    }
    else
    {
      publish_imu();
    }
    previous_imu_time = millis();
  }
  
  if(DEBUG)
  {
    if ((millis() - previous_debug_time) >= (1000 / DEBUG_RATE))
    {
      sprintf (buffer, "Encoder Left: %d", left_encoder.read());
      nh.loginfo(buffer);
      sprintf (buffer, "Encoder Right: %d", right_encoder.read());
      nh.loginfo(buffer);
      previous_debug_time = millis();
    }
  }


  nh.spinOnce();
}

void pid_callback( const lino_pid::linoPID& pid) 
{
  Kp = pid.p;
  Kd = pid.d;
  Ki = pid.i;
  sprintf (buffer, "P: %f D: %f D: %f", pid.p, pid.d, pid.i);
  nh.loginfo(buffer);
}

void command_callback( const geometry_msgs::Twist& cmd_msg)
{
  previous_command_time = millis();
  double linear_vel = cmd_msg.linear.x;
  double angular_vel = cmd_msg.angular.z;
  //convert m/s to m/min
  double linear_vel_mins = linear_vel * 60;
  //convert rad/s to rad/min
  double angular_vel_mins = angular_vel * 60;
  double circumference = pi * wheel_diameter;
  // Vt = ω * radius
  double tangential_vel = angular_vel_mins * (track_width / 2);

  left_motor.required_rpm = (linear_vel_mins / circumference) - (tangential_vel / circumference);
  right_motor.required_rpm = (linear_vel_mins / circumference) + (tangential_vel / circumference);
}

void drive_robot( int command_left, int command_right)
{
  //change left motor direction
  //forward
  if (command_left >= 0)
  {
    digitalWrite(left_motor_direction, HIGH);
  }
  //reverse
  else
  {
    digitalWrite(left_motor_direction, LOW);
  }
  analogWrite(left_motor_pwm, abs(command_left));
  
  //change right motor direction
  //forward
  if (command_right >= 0)
  {
    digitalWrite(right_motor_direction, HIGH);
  }
  //reverse
  else
  {
    digitalWrite(right_motor_direction, LOW);
  }

  analogWrite(right_motor_pwm, abs(command_right));
}

void read_motor_rpm_(Motor * mot, long current_encoder_ticks, unsigned long dt )
{
  dt = 60000 / dt;
  double delta_ticks = current_encoder_ticks - mot->previous_encoder_ticks;
  mot->current_rpm = (delta_ticks / double(encoder_pulse * gear_ratio)) * dt;
  mot->previous_encoder_ticks = current_encoder_ticks;
}

void publish_linear_velocity(unsigned long time)
{
  double average_rpm = (left_motor.current_rpm + right_motor.current_rpm) / 2; // rpm
  double average_rps = average_rpm / 60; // rps
  double linear_velocity = (average_rps * (wheel_diameter * pi)); // m/s 
     
  raw_vel_msg.header.stamp = nh.now();
  raw_vel_msg.vector.x = linear_velocity;
  raw_vel_msg.vector.y = 0.00;
  raw_vel_msg.vector.z = double(time) / 1000;
  raw_vel_pub.publish(&raw_vel_msg);
  nh.spinOnce();
}



void calculate_pwm(Motor * mot)
{
  double pid;
  double new_rpm;
  double error;
  
  error = mot->required_rpm - mot->current_rpm;
  mot->total_pid_error += error;
  pid = Kp * error  + Ki * mot->total_pid_error + Kd * (error - mot->previous_pid_error);
  mot->previous_pid_error = error;
  
  new_rpm = constrain(double(mot->pwm) * max_rpm / 255 + pid, -max_rpm, max_rpm);
  mot->pwm = (new_rpm / max_rpm) * 255;
}

void check_imu()
{
  raw_imu_msg.accelerometer = check_accelerometer();
  raw_imu_msg.gyroscope = check_gyroscope();
  raw_imu_msg.magnetometer = check_magnetometer();

  if (!raw_imu_msg.accelerometer)
  {
    nh.logerror("Accelerometer NOT FOUND!");
  }

  if (!raw_imu_msg.gyroscope)
  {
    nh.logerror("Gyroscope NOT FOUND!");
  }

  if (!raw_imu_msg.magnetometer)
  {
    nh.logerror("Magnetometer NOT FOUND!");
  }
  
  is_first = false;
}

void publish_imu()
{
  raw_imu_msg.header.stamp = nh.now();
  raw_imu_msg.header.frame_id = "imu_link";
  if (raw_imu_msg.accelerometer)
  {
    measure_acceleration();
    raw_imu_msg.raw_linear_acceleration = raw_acceleration;
  }

  if (raw_imu_msg.gyroscope)
  {
    measure_gyroscope();
    raw_imu_msg.raw_angular_velocity = raw_rotation;
  }

  if (raw_imu_msg.magnetometer)
  {
    measure_magnetometer();
    raw_imu_msg.raw_magnetic_field = raw_magnetic_field;
  }
  
  raw_imu_pub.publish(&raw_imu_msg);
}
