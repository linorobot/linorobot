#if (ARDUINO >= 100)
    #include <Arduino.h>
#else
    #include <WProgram.h>
#endif

#include <Servo.h>

#include "ros.h"
#include "ros/time.h"
//header file for publishing velocities for odom
#include "lino_msgs/Velocities.h"
//header file for cmd_subscribing to "cmd_vel"
#include "geometry_msgs/Twist.h"
//header file for pid server
#include "lino_msgs/PID.h"
//header file for imu
#include "lino_msgs/Imu.h"

#include "lino_base_config.h"
#include "Motor.h"
#include "Kinematics.h"
#include "PID.h"
#include "Imu.h"

#define ENCODER_OPTIMIZE_INTERRUPTS // comment this out on Non-Teensy boards
#include "Encoder.h"

#define IMU_PUBLISH_RATE 20 //hz
#define COMMAND_RATE 20 //hz
#define DEBUG_RATE 5

Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV);
Encoder motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV); 
Encoder motor3_encoder(MOTOR3_ENCODER_A, MOTOR3_ENCODER_B, COUNTS_PER_REV); 
Encoder motor4_encoder(MOTOR4_ENCODER_A, MOTOR4_ENCODER_B, COUNTS_PER_REV); 

Servo steering_servo;

Controller motor1_controller(Controller::MOTOR_DRIVER, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Controller motor2_controller(Controller::MOTOR_DRIVER, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B); 
Controller motor3_controller(Controller::MOTOR_DRIVER, MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B);
Controller motor4_controller(Controller::MOTOR_DRIVER, MOTOR4_PWM, MOTOR4_IN_A, MOTOR4_IN_B);

PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor3_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor4_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

Kinematics kinematics(Kinematics::LINO_BASE, MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE, LR_WHEELS_DISTANCE);

float g_req_linear_vel_x = 0;
float g_req_linear_vel_y = 0;
float g_req_angular_vel_z = 0;

unsigned long g_prev_command_time = 0;

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
    steering_servo.attach(STEERING_PIN);
    steering_servo.write(90); 
    
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

    //this block publishes the IMU data based on defined rate
    if ((millis() - prev_imu_time) >= (1000 / IMU_PUBLISH_RATE))
    {
        //sanity check if the IMU is connected
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
    motor3_pid.updateConstants(pid.p, pid.i, pid.d);
    motor4_pid.updateConstants(pid.p, pid.i, pid.d);
}

void commandCallback(const geometry_msgs::Twist& cmd_msg)
{
    //callback function every time linear and angular speed is received from 'cmd_vel' topic
    //this callback function receives cmd_msg object where linear and angular speed are stored
    g_req_linear_vel_x = cmd_msg.linear.x;
    g_req_linear_vel_y = cmd_msg.linear.y;
    g_req_angular_vel_z = cmd_msg.angular.z;

    g_prev_command_time = millis();
}


// Using current RPM as global variables. For debugging
int current_rpm1;
int current_rpm2;
int current_rpm3;
int current_rpm4;
int requested_current_rpm1;
int requested_current_rpm2;
int requested_current_rpm3;
int requested_current_rpm4;

void moveBase()
{
    //get the required rpm for each motor based on required velocities, and base used
    Kinematics::rpm req_rpm = kinematics.getRPM(g_req_linear_vel_x, g_req_linear_vel_y, g_req_angular_vel_z);

    //get the current speed of each motor and the requested. These are saved in a variable for debugging purposes
	// TEST 20190323, changed motor but is seems the rotation must sensing has to be reversed
	//current_rpm1 = motor1_encoder.getRPM();	
	current_rpm1 = -motor1_encoder.getRPM();	
	current_rpm2 = motor2_encoder.getRPM();
	current_rpm3 = motor3_encoder.getRPM();
    current_rpm4 = motor4_encoder.getRPM();
	requested_current_rpm1 = req_rpm.motor1;
	requested_current_rpm2 = req_rpm.motor2;
	requested_current_rpm3 = req_rpm.motor3;
	requested_current_rpm4 = req_rpm.motor4;
	
	// // HW FIX: 20190224
	// // Compensate for some different readings between sensors... This is a very hard fix...
	// // Claim to be verified
	// // Based on rotation sense has a different beahvior apparently
	// // If IMU calibration is reperformed, constant needs to be recalculated (count the number of ticks per turn of every wheel and optimize)
	// if (current_rpm1 > 0) {
		// current_rpm1 = int(current_rpm1*1.107562218); // It seems my sensor needs to be compensated. TODO: check if changing the sensor/Motor the situation improves
	// }
	// // Situation change when going backward... mmmm
	// if (current_rpm2 < 0) {
		// current_rpm2 = int(current_rpm2*2.041220769); // compensate for the quicker
	// }

    //the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
    //the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM
    motor1_controller.spin(motor1_pid.compute(requested_current_rpm1, current_rpm1));
    motor2_controller.spin(motor2_pid.compute(requested_current_rpm2, current_rpm2));
    motor3_controller.spin(motor3_pid.compute(requested_current_rpm3, current_rpm3));  
    motor4_controller.spin(motor4_pid.compute(requested_current_rpm4, current_rpm4));    

    Kinematics::velocities current_vel;

    if(kinematics.base_platform == Kinematics::ACKERMANN || kinematics.base_platform == Kinematics::ACKERMANN1)
    {
        float current_steering_angle;
        
        current_steering_angle = steer(g_req_angular_vel_z);
        current_vel = kinematics.getVelocities(current_steering_angle, current_rpm1, current_rpm2);
    }
    else
    {
        current_vel = kinematics.getVelocities(current_rpm1, current_rpm2, current_rpm3, current_rpm4);
    }
    
    //pass velocities to publisher object
    raw_vel_msg.linear_x = current_vel.linear_x;
    raw_vel_msg.linear_y = current_vel.linear_y;
    raw_vel_msg.angular_z = current_vel.angular_z;

    //publish raw_vel_msg
    raw_vel_pub.publish(&raw_vel_msg);
}

void stopBase()
{
    g_req_linear_vel_x = 0;
    g_req_linear_vel_y = 0;
    g_req_angular_vel_z = 0;
}

void publishIMU()
{
    //pass accelerometer data to imu object
    raw_imu_msg.linear_acceleration = readAccelerometer();

    //pass gyroscope data to imu object
    raw_imu_msg.angular_velocity = readGyroscope();

    //pass accelerometer data to imu object
    raw_imu_msg.magnetic_field = readMagnetometer();

    //publish raw_imu_msg
    raw_imu_pub.publish(&raw_imu_msg);
}

float steer(float steering_angle)
{
    //steering function for ACKERMANN base
    float servo_steering_angle;

    steering_angle = constrain(steering_angle, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE);
    servo_steering_angle = mapFloat(steering_angle, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE, PI, 0) * (180 / PI);

    steering_servo.write(servo_steering_angle);

    return steering_angle;
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


int32_t  prev_read_motor1_encoder;
int32_t  prev_read_motor2_encoder;
int32_t  prev_read_motor3_encoder;
int32_t  prev_read_motor4_encoder;

void printDebug()
{
    char buffer[100];
	
	int32_t read_motor1_encoder = motor1_encoder.read();
	int32_t read_motor2_encoder = motor2_encoder.read();
	int32_t read_motor3_encoder = motor3_encoder.read();
	int32_t read_motor4_encoder = motor4_encoder.read();
	geometry_msgs::Vector3 gyro = readGyroscope();
	geometry_msgs::Vector3 accel = readAccelerometer();
	
	// Requested Speed
	sprintf (buffer, "Required speed     : vel_x=%.2f, vel_y=%.2f, vel_z=%.2f", g_req_linear_vel_x, g_req_linear_vel_y, g_req_angular_vel_z);
    nh.loginfo(buffer);
	
	// ENCODER
    sprintf (buffer, "Encoder FrontLeft  : %ld \t diff:  %ld", read_motor1_encoder,  read_motor1_encoder - prev_read_motor1_encoder);
    nh.loginfo(buffer);
	sprintf (buffer, "Encoder FrontRight : %ld \t diff:  %ld", read_motor2_encoder,  read_motor2_encoder - prev_read_motor2_encoder);
    nh.loginfo(buffer);
	
	// RPMs
	sprintf (buffer, "Encoder FrontLeft  : Requested RPM %d \t Read RPM: %d \t  diff RPM:  %d", requested_current_rpm1, current_rpm1, requested_current_rpm1 - current_rpm1);
    nh.loginfo(buffer);
	sprintf (buffer, "Encoder FrontRight : Requested RPM %d \t Read RPM: %d \t  diff RPM:  %d", requested_current_rpm2, current_rpm2, requested_current_rpm2 - current_rpm2);
    nh.loginfo(buffer);
	
	//IMU
	sprintf (buffer, "IMU Readings Accel : a_x=%2.2f, a_y=%2.2f, a_z=%2.2f", accel.x, accel.y, accel.z);
	nh.loginfo(buffer);
	sprintf (buffer, "IMU Readings Gyro  : g_x=%2.2f, g_y=%2.2f, g_z=%2.2f", gyro.x, gyro.y, gyro.z);
    nh.loginfo(buffer);
	
    //sprintf (buffer, "Encoder RearLeft   : %ld - RPM: %d - diff:  %ld", read_motor3_encoder, motor3_encoder.getRPM(), read_motor3_encoder - prev_read_motor3_encoder);
    //nh.loginfo(buffer);
    //sprintf (buffer, "Encoder RearRight  : %ld - RPM: %d - diff:  %ld", read_motor4_encoder, motor4_encoder.getRPM(), read_motor4_encoder - prev_read_motor4_encoder);
    //nh.loginfo(buffer);
	
	prev_read_motor1_encoder = read_motor1_encoder;
	prev_read_motor2_encoder = read_motor2_encoder;
	prev_read_motor3_encoder = read_motor3_encoder;
	prev_read_motor4_encoder = read_motor4_encoder;
}
