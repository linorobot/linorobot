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


#define IMU_PUBLISH_RATE 5 //hz
#define COMMAND_RATE 8 //hz Number of time per second the velocity command is parsed. Used to filter out too many command to the ESC
#define DEBUG_RATE 5 // mess per second

#ifdef USE_ESC // Assuming using ESC uses needs to extract the RPM in the 
#include "Encoder_BLDC.h"
//Setup a pointer for the encoders needed, this has to be init with proper pins in the setup
Encoder_BLDC motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, MOTOR1_ENCODER_C, COUNTS_PER_REV);
Encoder_BLDC motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, MOTOR2_ENCODER_C, COUNTS_PER_REV);

//Encapsulate a call for any interrupts to the relative encoder class, this has to be asssociated in the setup
void enc1InterruptHandler_A(void) { motor1_encoder.handleInterrupt_U(); }
void enc1InterruptHandler_B(void) { motor1_encoder.handleInterrupt_V(); }
void enc1InterruptHandler_C(void) { motor1_encoder.handleInterrupt_W(); }
void enc2InterruptHandler_A(void) { motor2_encoder.handleInterrupt_U(); }
void enc2InterruptHandler_B(void) { motor2_encoder.handleInterrupt_V(); }
void enc2InterruptHandler_C(void) { motor2_encoder.handleInterrupt_W(); }

#if LINO_BASE==1 // as defined in Kinematics.h --->>> {DIFFERENTIAL_DRIVE, SKID_STEER, ACKERMANN, ACKERMANN1, MECANUM};
Encoder_BLDC motor3_encoder(MOTOR3_ENCODER_A, MOTOR3_ENCODER_B,MOTOR4_ENCODER_C, COUNTS_PER_REV);
Encoder_BLDC motor4_encoder(MOTOR4_ENCODER_A, MOTOR4_ENCODER_B,MOTOR4_ENCODER_C, COUNTS_PER_REV);
void enc3InterruptHandler_A(void) { motor3_encoder.handleInterrupt_U(); }
void enc3InterruptHandler_B(void) { motor3_encoder.handleInterrupt_V(); }
void enc3InterruptHandler_C(void) { motor3_encoder.handleInterrupt_W(); }
void enc4InterruptHandler_A(void) { motor4_encoder.handleInterrupt_U(); }
void enc4InterruptHandler_B(void) { motor4_encoder.handleInterrupt_V(); }
void enc4InterruptHandler_C(void) { motor4_encoder.handleInterrupt_W(); }

#endif //LINO_BASE
#else
#include "Encoder.h"
Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV);
Encoder motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV); 
#if LINO_BASE==1 // as defined in Kinematics.h --->>> {DIFFERENTIAL_DRIVE, SKID_STEER, ACKERMANN, ACKERMANN1, MECANUM};
Encoder motor3_encoder(MOTOR3_ENCODER_A, MOTOR3_ENCODER_B, COUNTS_PER_REV); 
Encoder motor4_encoder(MOTOR4_ENCODER_A, MOTOR4_ENCODER_B, COUNTS_PER_REV); 
#endif //LINO_BASE
#endif//USE_ESC
// as defined in Kinematics.h --->>> {DIFFERENTIAL_DRIVE, SKID_STEER, ACKERMANN, ACKERMANN1, MECANUM};
#if LINO_BASE==2 || LINO_BASE==3
Servo steering_servo;
#endif 

Controller motor1_controller(Controller::MOTOR_DRIVER, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Controller motor2_controller(Controller::MOTOR_DRIVER, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B); 
#if LINO_BASE==1 // as defined in Kinematics.h --->>> {DIFFERENTIAL_DRIVE, SKID_STEER, ACKERMANN, ACKERMANN1, MECANUM};
Controller motor3_controller(Controller::MOTOR_DRIVER, MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B);
Controller motor4_controller(Controller::MOTOR_DRIVER, MOTOR4_PWM, MOTOR4_IN_A, MOTOR4_IN_B);
#endif

PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
#if LINO_BASE==1
PID motor3_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor4_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
#endif

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
// as defined in Kinematics.h --->>> {DIFFERENTIAL_DRIVE, SKID_STEER, ACKERMANN, ACKERMANN1, MECANUM};
#if LINO_BASE==2 || LINO_BASE==3
    steering_servo.attach(STEERING_PIN);
    steering_servo.write(90); 
#endif

    nh.initNode();
    nh.getHardware()->setBaud(57600);
    nh.subscribe(pid_sub);
    nh.subscribe(cmd_sub);
    nh.advertise(raw_vel_pub);
    nh.advertise(raw_imu_pub);

// Init encoder
#ifdef USE_ESC
	motor1_encoder.setupInterruptHandler(MOTOR1_ENCODER_A, enc1InterruptHandler_A, CHANGE);
	motor1_encoder.setupInterruptHandler(MOTOR1_ENCODER_B, enc1InterruptHandler_B, CHANGE);
	motor1_encoder.setupInterruptHandler(MOTOR1_ENCODER_C, enc1InterruptHandler_C, CHANGE);
	motor2_encoder.setupInterruptHandler(MOTOR2_ENCODER_A, enc2InterruptHandler_A, CHANGE);
	motor2_encoder.setupInterruptHandler(MOTOR2_ENCODER_B, enc2InterruptHandler_B, CHANGE);
	motor2_encoder.setupInterruptHandler(MOTOR2_ENCODER_C, enc2InterruptHandler_C, CHANGE);
#if LINO_BASE==1 // as defined in Kinematics.h --->>> {DIFFERENTIAL_DRIVE, SKID_STEER, ACKERMANN, ACKERMANN1, MECANUM};
	motor3_encoder.setupInterruptHandler(MOTOR3_ENCODER_A, enc1InterruptHandler_A, CHANGE);
	motor3_encoder.setupInterruptHandler(MOTOR3_ENCODER_B, enc1InterruptHandler_B, CHANGE);
	motor3_encoder.setupInterruptHandler(MOTOR3_ENCODER_C, enc1InterruptHandler_C, CHANGE);
	motor4_encoder.setupInterruptHandler(MOTOR4_ENCODER_A, enc2InterruptHandler_A, CHANGE);
	motor4_encoder.setupInterruptHandler(MOTOR4_ENCODER_B, enc2InterruptHandler_B, CHANGE);
	motor4_encoder.setupInterruptHandler(MOTOR4_ENCODER_C, enc2InterruptHandler_C, CHANGE);
#endif
#endif	
	
    while (!nh.connected())
    {
        nh.spinOnce();
    }
    nh.loginfo("LINOBASE CONNECTED");
    delay(1);
// as defined in Kinematics.h --->>> {DIFFERENTIAL_DRIVE, SKID_STEER, ACKERMANN, ACKERMANN1, MECANUM};
	#if LINO_BASE==0
		nh.loginfo("LINOBASE configured as DIFFERENTIAL_DRIVE (2WD)");
	#elif LINO_BASE==1
		nh.loginfo("LINOBASE configured as SKID_STEER (4WD)");
	#elif LINO_BASE==2 || LINO_BASE==3
		nh.loginfo("LINOBASE configured as ACKERMANN");
	#else
		nh.loginfo("LINOBASE UNKNOWN configuration"); //TODO: MECCANO
	#endif
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
// as defined in Kinematics.h --->>> {DIFFERENTIAL_DRIVE, SKID_STEER, ACKERMANN, ACKERMANN1, MECANUM};
#if LINO_BASE==1
    motor3_pid.updateConstants(pid.p, pid.i, pid.d);
    motor4_pid.updateConstants(pid.p, pid.i, pid.d);
#endif
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
int requested_current_rpm1;
int requested_current_rpm2;
int computed_pwm_1;
int computed_pwm_2;
int applied_pwm_motor1;
int applied_pwm_motor2;
// as defined in Kinematics.h --->>> {DIFFERENTIAL_DRIVE, SKID_STEER, ACKERMANN, ACKERMANN1, MECANUM};
#if LINO_BASE==1
int current_rpm3;
int current_rpm4;
int requested_current_rpm3;
int requested_current_rpm4;
int computed_pwm_3;
int computed_pwm_4;
int applied_pwm_motor3;
int applied_pwm_motor4;
#endif

int wait_delay;
void moveBase()
{
	static int delay_btw_spin_motor_1 = 0; 
	static int delay_btw_spin_motor_2 = 0; 
	static int delay_btw_spin_motor_3 = 0; 
	static int delay_btw_spin_motor_4 = 0; 
	wait_delay = 0;
    //get the required rpm for each motor based on required velocities, and base used
    Kinematics::rpm req_rpm = kinematics.getRPM(g_req_linear_vel_x, g_req_linear_vel_y, g_req_angular_vel_z);

    //get the current speed of each motor and the requested. These are saved in a variable for debugging purposes
	// One motor is mounted inverted, motor 2 in this case	
	current_rpm1 = motor1_encoder.getRPM();	
	current_rpm2 = -motor2_encoder.getRPM(); // The motor is rotating inverted
	requested_current_rpm1 = req_rpm.motor1;
	requested_current_rpm2 = req_rpm.motor2;
#if LINO_BASE==1
	current_rpm3 = motor3_encoder.getRPM();
    current_rpm4 = motor4_encoder.getRPM();
	requested_current_rpm3 = req_rpm.motor3;
	requested_current_rpm4 = req_rpm.motor4;
#endif
    //the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
    //the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM
	computed_pwm_1 = motor1_pid.compute(requested_current_rpm1, current_rpm1);
	computed_pwm_2 = motor2_pid.compute(-requested_current_rpm2, -current_rpm2); // The motor 2 must rotate inverted
#ifdef USE_ESC
	applied_pwm_motor1 = mapESC_PWM(computed_pwm_1); // needed to map an eventual min PWM 
	applied_pwm_motor2 = mapESC_PWM(computed_pwm_2);
#endif
	delay_btw_spin_motor_1 = motor1_controller.spin(applied_pwm_motor1);
    delay_btw_spin_motor_2 = motor2_controller.spin(applied_pwm_motor2);
#if LINO_BASE==1 // CODE NOT MODIFIED SEE LINES ABOVE WITH applied_pwm_motor1
    delay_btw_spin_motor_3 = motor3_controller.spin(motor3_pid.compute(requested_current_rpm3, current_rpm3));  
    delay_btw_spin_motor_4 = motor4_controller.spin(motor4_pid.compute(requested_current_rpm4, current_rpm4));    
#endif
	// if some of the spin has to go to 0 wait until it has finished. Should all the spin set to 0 and then set again
	wait_delay = max(delay_btw_spin_motor_1, max(delay_btw_spin_motor_2, max(delay_btw_spin_motor_3, delay_btw_spin_motor_4)));
	if (wait_delay) {
		motor1_controller.spin(0);
		motor2_controller.spin(0);
		delay(wait_delay);
		motor1_controller.spin(applied_pwm_motor1);
		motor2_controller.spin(applied_pwm_motor2);
	}
	
#if LINO_BASE==1
	// CODE NOT MODIFIED SEE LINES ABOVE WITH wait_delay
    motor3_controller.spin(motor3_pid.compute(requested_current_rpm3, current_rpm3));  
    motor4_controller.spin(motor4_pid.compute(requested_current_rpm4, current_rpm4));    
#endif    
	
	Kinematics::velocities current_vel;
	
// as defined in Kinematics.h --->>> {DIFFERENTIAL_DRIVE, SKID_STEER, ACKERMANN, ACKERMANN1, MECANUM};
#if LINO_BASE==2 || LINO_BASE==3
	// NEVER TESTED!!
	// ACKERMANN
       float current_steering_angle;
       current_steering_angle = steer(g_req_angular_vel_z);
       current_vel = kinematics.getVelocities(current_steering_angle, current_rpm1, current_rpm2);
#elif LINO_BASE==1
	//4WD CASE
		current_vel = kinematics.getVelocities(current_rpm1, current_rpm2, current_rpm3,current_rpm4);
#else
	//2WD CASE
		current_vel = kinematics.getVelocities(current_rpm1, current_rpm2, 0,0);// Motor 3 and 4 are not considered in this kinematics, TODO: function should be overloaded in kinematics!
#endif
//	TODO: MECCANO IS MISSING
    
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
// as defined in Kinematics.h --->>> {DIFFERENTIAL_DRIVE, SKID_STEER, ACKERMANN, ACKERMANN1, MECANUM};
#if LINO_BASE==2 || LINO_BASE==3
float steer(float steering_angle)
{
    //steering function for ACKERMANN base
    float servo_steering_angle;

    steering_angle = constrain(steering_angle, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE);
    servo_steering_angle = mapFloat(steering_angle, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE, PI, 0) * (180 / PI);

    steering_servo.write(servo_steering_angle);

    return steering_angle;
}
#endif

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#ifdef USE_ESC 
int mapESC_PWM(int pwm) {
	int ret_pwm = pwm;
	if (pwm > 0) 	  { ret_pwm = map(pwm, 0       , PWM_MAX , PWM_MIN_THRESHOLD, PWM_MAX           );}
	else if (pwm < 0) { ret_pwm = map(pwm, PWM_MIN , 0       , PWM_MIN          , -PWM_MIN_THRESHOLD); }	
	return ret_pwm;
}
#endif

int32_t  prev_read_motor1_encoder;
int32_t  prev_read_motor2_encoder;
int32_t read_motor1_encoder;
int32_t read_motor2_encoder;
#if LINO_BASE==1
int32_t  prev_read_motor3_encoder;
int32_t  prev_read_motor4_encoder;
#endif

void printDebug()
{
    static char buffer[150];
	
	read_motor1_encoder = motor1_encoder.read();
	read_motor2_encoder = motor2_encoder.read();
	//applied_pwm_motor1 = motor1_controller.latest_spin_applied();
	//applied_pwm_motor2 = motor2_controller.latest_spin_applied();
	//int32_t read_motor3_encoder = motor3_encoder.read();
	//int32_t read_motor4_encoder = motor4_encoder.read();
	static geometry_msgs::Vector3 gyro = readGyroscope();
	static geometry_msgs::Vector3 accel = readAccelerometer();
// #ifdef USE_ESC
	// static float* read_timing1 = motor1_encoder.read_timing();
	// static float* read_timing2 = motor2_encoder.read_timing();
	//// Timing Sensors
	// sprintf (buffer, "Timing sensor 1    : avg=%.4f, U=%.4f, V=%.4f, W=%.4f", read_timing1[0], read_timing1[1], read_timing1[2], read_timing1[3]);
	// nh.loginfo(buffer);
	// sprintf (buffer, "Timing sensor 2    : avg=%.4f, U=%.4f, V=%.4f, W=%.4f", read_timing2[0], read_timing2[1], read_timing2[2], read_timing2[3]);
    // nh.loginfo(buffer);
// #endif	
	
	// Requested Speed
	sprintf (buffer, "Required speed     : vel_x=%.2f, vel_y=%.2f, vel_z=%.2f", g_req_linear_vel_x, g_req_linear_vel_y, g_req_angular_vel_z);
    nh.loginfo(buffer);
	
	// PWM generated
	sprintf (buffer, "Calucated(applied) PWM speed : PWM_FL=%d(%d), PWM_FR=%d(%d) - waitdelay %d ms", computed_pwm_1, applied_pwm_motor1, computed_pwm_2,applied_pwm_motor2, wait_delay);
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

//	prev_read_motor3_encoder = read_motor3_encoder;
//	prev_read_motor4_encoder = read_motor4_encoder;

}