#include <ArduinoLog.h>

// RATE AND TIMING
#define COMMAND_RATE 4 //hz Number of time per second the velocity command is parsed. Used to filter out too many command to the ESC
#define DEBUG_RATE   8 //hz, nuber of print per second NOTE: to consider the rotation of the encoder

// TODO: used to debug encoder
#define BLDC_ENC_DEBUG 1

// Serial Command
#define SERIAL_BAUD 57600
#include <SoftwareSerial.h>   // We need this even if we're not using a SoftwareSerial object
                              // Due to the way the Arduino IDE compiles
#include <SerialCommand.h>
SerialCommand SCmd; // The demo SerialCommand object

// ------------------ ENCODER -----------------
#define MOTOR1_ENCODER_A 15
#define MOTOR1_ENCODER_B 14 
#define MOTOR1_ENCODER_C 20 

#define MOTOR2_ENCODER_A 10 //11
#define MOTOR2_ENCODER_B 11 //10 
#define MOTOR2_ENCODER_C 6 

// ------------------ MOTOR -----------------
#define MOTOR_DRIVER ESC  
#define MOTOR1_PWM 1 //DON'T TOUCH THIS! This is just a placeholder
#define MOTOR1_IN_A 21
#define MOTOR1_IN_B 20 // Not use in case of ESC. Only for compatibility, PIN is reused for ENCODER!!

#define MOTOR2_PWM 8 //DON'T TOUCH THIS! This is just a placeholder
#define MOTOR2_IN_A 5
#define MOTOR2_IN_B 6 // Not use in case of ESC. Only for compatibility, PIN is reused for ENCODER!!

#define PWM_MAX 400
#define PWM_MIN -PWM_MAX
#define PWM_TH_POSITIVE 50
#define PWM_TH_NEGATIVE -PWM_TH_POSITIVE

// ------------------ OTHER CONFIG -----------------
#define MAX_RPM 2000               // motor's maximum RPM
#define COUNTS_PER_REV 6*5       // wheel encoder's no of ticks per rev (e.g motors for one rotation should be 6, then multiplz for the number of the ration e.g 1:6 is 36 counts per rev???)
#define DELAY_REVERSE 1000    //  ms waitint when reversing PWM . 1000 is Safe, find empirical 500 ms sometime doesnt reverse properly

#include "Encoder_BLDC.h" // enabling this i get an error!
#include "Motor.h"


// ------------------ ENCODER SETUP -----------------
Encoder_BLDC motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, MOTOR1_ENCODER_C, COUNTS_PER_REV);
Encoder_BLDC motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, MOTOR2_ENCODER_C, COUNTS_PER_REV);

//Encapsulate a call for any interrupts to the relative encoder class, this has to be asssociated in the setup
void enc1InterruptHandler_A(void) { motor1_encoder.handleInterrupt_U(); }
void enc1InterruptHandler_B(void) { motor1_encoder.handleInterrupt_V(); }
void enc1InterruptHandler_C(void) { motor1_encoder.handleInterrupt_W(); }
void enc2InterruptHandler_A(void) { motor2_encoder.handleInterrupt_U(); }
void enc2InterruptHandler_B(void) { motor2_encoder.handleInterrupt_V(); }
void enc2InterruptHandler_C(void) { motor2_encoder.handleInterrupt_W(); }


// ------------------ MOTOR SETUP SETUP -----------------
Controller motor1_controller(Controller::MOTOR_DRIVER, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Controller motor2_controller(Controller::MOTOR_DRIVER, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B); 

// save the latest PWM received
static int lastSetPWM = 0;

void setup() {
  // Init Serial
  Serial.begin(SERIAL_BAUD);
  // Initialize with log level and log output. 
  Log.begin   (LOG_LEVEL_VERBOSE, &Serial);
  Log.notice( "***\t\tLogging ESC_And_Encoder\t\t***" CR); 
  Log.notice( "***\t\tSerial BAUD %d \t\t***" CR, SERIAL_BAUD); 
  // In Monitor use end line Both NL&CR
  SCmd.addCommand("PWM",CMD_moveBase);        // return the vector with the relative angles
  SCmd.addCommand("STOP",CMD_stop);        // return the vector with the relative angles
  SCmd.addDefaultHandler(CMD_unrecognized);  // Handler for command that isn't matched  (says "What?") 

  // Add interrupt function for encoders 
  motor1_encoder.setupInterruptHandler(MOTOR1_ENCODER_A, enc1InterruptHandler_A, CHANGE);
  motor1_encoder.setupInterruptHandler(MOTOR1_ENCODER_B, enc1InterruptHandler_B, CHANGE);
  motor1_encoder.setupInterruptHandler(MOTOR1_ENCODER_C, enc1InterruptHandler_C, CHANGE);
  motor2_encoder.setupInterruptHandler(MOTOR2_ENCODER_A, enc2InterruptHandler_A, CHANGE);
  motor2_encoder.setupInterruptHandler(MOTOR2_ENCODER_B, enc2InterruptHandler_B, CHANGE);
  motor2_encoder.setupInterruptHandler(MOTOR2_ENCODER_C, enc2InterruptHandler_C, CHANGE);

}

void loop() {
  static unsigned long prev_control_time = 0;
  static unsigned long prev_debug_time = 0;
  static String command;
  
  SCmd.readSerial();
  if ((millis() - prev_control_time) >= (1000 / COMMAND_RATE)) {
      moveBase(lastSetPWM);
      prev_control_time = millis();
  }
  if ((millis() - prev_debug_time) >= (1000 / DEBUG_RATE)) {
      printDebug();
      prev_debug_time = millis();
  } 
}

// ------------------ SERIAL COMMAND -----------------
void CMD_unrecognized(){
  Log.notice( "Unknown command" CR); 
} 

void CMD_moveBase() {
  const char *command = SCmd.next();
  lastSetPWM = atoi(command);
  Log.notice( "CMD_moveBase %s, %d" CR, command, lastSetPWM); 
  moveBase(lastSetPWM);
}

void CMD_stop(){
  lastSetPWM = 0;
  Log.notice( "CMD_stop" CR); 
  moveBase(lastSetPWM);
} 

// From the original move base, PWM is set directly
static int computed_pwm_1;
static int computed_pwm_2;
static int current_rpm1;
static int current_rpm2;
//static int requested_current_rpm1;
//static int requested_current_rpm2;
void moveBase(int set_pwm)
{
  static bool reversed = 0;
  //get the required rpm for each motor based on required velocities, and base used
  // Kinematics::rpm req_rpm = kinematics.getRPM(g_req_linear_vel_x, g_req_linear_vel_y, g_req_angular_vel_z);
  //the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
  //the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM
  //computed_pwm_1 = motor1_pid.compute(requested_current_rpm1, current_rpm1);
  //computed_pwm_2 = -motor2_pid.compute(requested_current_rpm2, -current_rpm2);
  //get the current speed of each motor and the requested. These are saved in a variable for debugging purposes
  // TEST 20190323, changed motor but is seems the rotation must sensing has to be reversed
  //current_rpm1 = motor1_encoder.getRPM(); 
  current_rpm1 = motor1_encoder.getRPM(); 
  current_rpm2 = -motor2_encoder.getRPM();
  //requested_current_rpm1 = req_rpm.motor1;
  //requested_current_rpm2 = req_rpm.motor2;

  //the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
  //the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM
  //computed_pwm_1 = motor1_pid.compute(requested_current_rpm1, current_rpm1);
  //computed_pwm_2 = -motor2_pid.compute(requested_current_rpm2, -current_rpm2);
  
  // Check if the PWM must be reversed
  if (set_pwm*computed_pwm_1 < 0) { // sign change
    motor1_controller.spin(0); // Stop the spin
    reversed = 1;
  }
  if (-set_pwm*computed_pwm_2 < 0) { // sign change
    motor2_controller.spin(0); // Stop the spin
    reversed = 1;
  }
  if (reversed) delay(DELAY_REVERSE);

  // Set the minimum threshold PWM 
  if (set_pwm > 0) {
    set_pwm = max(set_pwm, PWM_TH_POSITIVE);
  }
  if (set_pwm < 0) {
    set_pwm = min(set_pwm, PWM_TH_NEGATIVE);
  }
  
  computed_pwm_1 = set_pwm;
  computed_pwm_2 = -set_pwm;

  motor1_controller.spin(computed_pwm_1);
  motor2_controller.spin(computed_pwm_2);
  reversed = 0;
}

// Other
void printDebug()
{
  static char buffer[150];
  static int32_t  prev_read_motor1_encoder;
  static int32_t  prev_read_motor2_encoder;
  static int32_t  read_motor1_encoder;
  static int32_t  read_motor2_encoder;

  read_motor1_encoder = motor1_encoder.read();
  read_motor2_encoder = motor2_encoder.read();
  
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
  // sprintf (buffer, "Required speed     : vel_x=%.2f, vel_y=%.2f, vel_z=%.2f", g_req_linear_vel_x, g_req_linear_vel_y, g_req_angular_vel_z);
  // nh.loginfo(buffer);
  
  // PWM generated
  sprintf (buffer, "Calucated PWM speed : PWM_FL=%d, PWM_FR=%d" CR, computed_pwm_1, computed_pwm_2);
  // nh.loginfo(buffer);
  Log.notice(buffer);
  
  // ENCODER
  sprintf (buffer, "Encoder FrontLeft  : %ld \t diff:  %ld" CR, read_motor1_encoder,  read_motor1_encoder - prev_read_motor1_encoder);
  // nh.loginfo(buffer);
  Log.notice(buffer);
  sprintf (buffer, "Encoder FrontRight : %ld \t diff:  %ld" CR, read_motor2_encoder,  read_motor2_encoder - prev_read_motor2_encoder);
  // nh.loginfo(buffer);
  Log.notice(buffer);
  
  // RPMs
  //sprintf (buffer, "Encoder FrontLeft  : Requested RPM %d \t Read RPM: %d \t  diff RPM:  %d", requested_current_rpm1, current_rpm1, requested_current_rpm1 - current_rpm1);
  //nh.loginfo(buffer);
  //sprintf (buffer, "Encoder FrontRight : Requested RPM %d \t Read RPM: %d \t  diff RPM:  %d", requested_current_rpm2, current_rpm2, requested_current_rpm2 - current_rpm2);
  //nh.loginfo(buffer);
  sprintf (buffer, "Encoder Front  : Left RPM %d \t Right RPM: %d " CR, current_rpm1, current_rpm2);
  // nh.loginfo(buffer);
  Log.notice(buffer);
  
  //IMU
//  sprintf (buffer, "IMU Readings Accel : a_x=%2.2f, a_y=%2.2f, a_z=%2.2f", accel.x, accel.y, accel.z);
//  nh.loginfo(buffer);
//  sprintf (buffer, "IMU Readings Gyro  : g_x=%2.2f, g_y=%2.2f, g_z=%2.2f", gyro.x, gyro.y, gyro.z);
//    nh.loginfo(buffer);
  
    //sprintf (buffer, "Encoder RearLeft   : %ld - RPM: %d - diff:  %ld", read_motor3_encoder, motor3_encoder.getRPM(), read_motor3_encoder - prev_read_motor3_encoder);
    //nh.loginfo(buffer);
    //sprintf (buffer, "Encoder RearRight  : %ld - RPM: %d - diff:  %ld", read_motor4_encoder, motor4_encoder.getRPM(), read_motor4_encoder - prev_read_motor4_encoder);
    //nh.loginfo(buffer);
  prev_read_motor1_encoder = read_motor1_encoder;
  prev_read_motor2_encoder = read_motor2_encoder;
}
