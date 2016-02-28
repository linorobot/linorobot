
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
  double previous_pid_error; //last measured pid error
  double total_pid_error; //overall pid error of the motor
  long previous_encoder_ticks; //last measured encoder ticks
  double current_rpm; //current speed of the motor
  double required_rpm;//desired speed of the motor 
  int pwm;//desired speed of the motor mapped to PWM
}
Motor;

//create a Motor object
Motor left_motor;
Motor right_motor;

//create an Encoder object
Encoder left_encoder(left_encoder_a, left_encoder_b);
Encoder right_encoder(right_encoder_a, right_encoder_b);

//function prototypes
void drive_robot(int, int);
void check_imu();
void publish_imu();
void publish_linear_velocity(unsigned long);
void read_motor_rpm_(Motor * mot, long current_encoder_ticks, unsigned long dt );
void calculate_pwm(Motor * mot);

//callback function prototypes
void command_callback( const geometry_msgs::Twist& cmd_msg);
void pid_callback( const lino_pid::linoPID& pid);

unsigned long lastMilli = 0;       
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
  //this block publishes velocity based on defined rate
  if ((millis() - publish_vel_time) >= (1000 / VEL_PUBLISH_RATE))
  {
    unsigned long current_time = millis();
    publish_linear_velocity(current_time - publish_vel_time);
    publish_vel_time = millis();
  }

  //this block drives the robot based on defined rate
  if ((millis() - previous_control_time) >= (1000 / COMMAND_RATE))
  {
    unsigned long current_time = millis();
    unsigned long dt = current_time - previous_control_time;
    //calculate motor's current speed
    read_motor_rpm_(&left_motor, left_encoder.read(), dt);
    read_motor_rpm_(&right_motor, right_encoder.read(), dt);
    //calculate how much PWM is needed based on required RPM
    calculate_pwm(&left_motor);
    calculate_pwm(&right_motor);    
    //move the wheels based on the PWM calculated
    drive_robot(left_motor.pwm, right_motor.pwm);
    previous_control_time = millis();
  }

  //this block stops the motor when no command is received
  if ((millis() - previous_command_time) >= 400)
  {
    left_motor.required_rpm = 0;
    right_motor.required_rpm = 0;
    right_motor.pwm = 0;
    left_motor.pwm = 0;
    drive_robot(left_motor.pwm, right_motor.pwm);
  }

  //this block publishes the IMU data based on defined rate
  if ((millis() - previous_imu_time) >= (1000 / IMU_PUBLISH_RATE))
  {
    //sanity check if the IMU exits
    if (is_first)
    {
      check_imu();
    }
    else
    {
      //publish the IMU data
      publish_imu();
    }
    previous_imu_time = millis();
  }
  
  //this block displays the encoder readings. change DEBUG to 0 if you don't want to display
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

  //call all the callbacks waiting to be called
  nh.spinOnce();
}

void pid_callback( const lino_pid::linoPID& pid) 
{
  //callback function every time PID constants are received from lino_pid for tuning
  //this callback receives pid object where P,I, and D constants are stored
  Kp = pid.p;
  Kd = pid.d;
  Ki = pid.i;
  sprintf (buffer, "P: %f D: %f D: %f", pid.p, pid.d, pid.i);
  nh.loginfo(buffer);
}

void command_callback( const geometry_msgs::Twist& cmd_msg)
{
  //callback function every time linear and angular speed is received from 'cmd_vel' topic
  //this callback function receives cmd_msg object where linear and angular speed are stored

  previous_command_time = millis();
  double linear_vel = cmd_msg.linear.x;
  double angular_vel = cmd_msg.angular.z;
  //convert m/s to m/min
  double linear_vel_mins = linear_vel * 60;
  //convert rad/s to rad/min
  double angular_vel_mins = angular_vel * 60;
  //calculate the wheel's circumference
  double circumference = pi * wheel_diameter;
  //calculate the tangential velocity of the wheel if the robot's rotating where Vt = ω * radius
  double tangential_vel = angular_vel_mins * (track_width / 2);

  //calculate and assign desired RPM for each motor
  left_motor.required_rpm = (linear_vel_mins / circumference) - (tangential_vel / circumference);
  right_motor.required_rpm = (linear_vel_mins / circumference) + (tangential_vel / circumference);
}

void drive_robot( int command_left, int command_right)
{
  //this functions spins the left and right wheel based on a defined speed in PWM  
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
  //spin the motor
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
  //spin the motor
  analogWrite(right_motor_pwm, abs(command_right));
}

void read_motor_rpm_(Motor * mot, long current_encoder_ticks, unsigned long dt )
{
  // this function calculates the motor's RPM based on encoder ticks and delta time
  
  //convert the time from milliseconds to minutes
  dt = 60000 / dt;
  //calculate change in number of ticks from the encoder
  double delta_ticks = current_encoder_ticks - mot->previous_encoder_ticks;
  //calculate wheel's speed (RPM)
  mot->current_rpm = (delta_ticks / double(encoder_pulse * gear_ratio)) * dt;
  mot->previous_encoder_ticks = current_encoder_ticks;
}

void publish_linear_velocity(unsigned long time)
{
  // this function publishes the linear speed of the robot
  
  //calculate the average RPM 
  double average_rpm = (left_motor.current_rpm + right_motor.current_rpm) / 2; // RPM
  //convert revolutions per minute to revolutions per second
  double average_rps = average_rpm / 60; // RPS
  //calculate linear speed
  double linear_velocity = (average_rps * (wheel_diameter * pi)); // m/s 
  
  //fill in the object 
  raw_vel_msg.header.stamp = nh.now();
  raw_vel_msg.vector.x = linear_velocity;
  raw_vel_msg.vector.y = 0.00;
  raw_vel_msg.vector.z = double(time) / 1000;
  //publish raw_vel_msg object to ROS
  raw_vel_pub.publish(&raw_vel_msg);
  nh.spinOnce();
}

void calculate_pwm(Motor * mot)
{
  // this function takes a Motor object argument,
  // implements a PID controller and calculates the PWM required to drive the motor
  double pid;
  double new_rpm;
  double error;
  
  //calculate the error ()
  error = mot->required_rpm - mot->current_rpm;
  //calculate the overall error
  mot->total_pid_error += error;
  //PID controller
  pid = Kp * error  + Ki * mot->total_pid_error + Kd * (error - mot->previous_pid_error);
  mot->previous_pid_error = error;
  //adds the calculated PID value to the required rpm for error compensation
  new_rpm = constrain(double(mot->pwm) * max_rpm / 255 + pid, -max_rpm, max_rpm);
  //maps rpm to PWM signal, where 255 is the max possible value from an 8 bit controller
  mot->pwm = (new_rpm / max_rpm) * 255;
}

void check_imu()
{
  //this function checks if IMU is present
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
  //this function publishes raw IMU reading
  raw_imu_msg.header.stamp = nh.now();
  raw_imu_msg.header.frame_id = "imu_link";
  //measure accelerometer
  if (raw_imu_msg.accelerometer)
  {
    measure_acceleration();
    raw_imu_msg.raw_linear_acceleration = raw_acceleration;
  }

  //measure gyroscope
  if (raw_imu_msg.gyroscope)
  {
    measure_gyroscope();
    raw_imu_msg.raw_angular_velocity = raw_rotation;
  }
  
  //measure magnetometer
  if (raw_imu_msg.magnetometer)
  {
    measure_magnetometer();
    raw_imu_msg.raw_magnetic_field = raw_magnetic_field;
  }
  //publish raw_imu_msg object to ROS
  raw_imu_pub.publish(&raw_imu_msg);
}
