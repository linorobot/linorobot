# kinematics
Arduino Kinematics library for differential drive(2WD, 4WD) and mecanum drive robots.

The library requires the following robot's specification as an input:

  - Robot's maximum RPM
  - Distance between wheels (base width)
  - Wheel's Diameter

## Functions

#### 1. Kinematics::Kinematics(int motor_max_rpm, float wheel_diameter, float base_width, int pwm_bits)
Object constructor which requires the robot's specification.
  - motor_max_rpm : Maximum RPM of the motor
  - wheel_diameter : Robot wheel's diameter expressed in meters
  - base_width : Distance between wheels
  - pwm_bits : PWM resolution of the Microcontroller. Arduino Uno/Mega, Teensy is 8 bits by default.

#### 2.  output getRPM(float linear_x, float linear_y, float angular_z)
Returns a Vector of Motor RPMs from a given linear velocity in x and y axis and angular velocity in z axis using right hand rule. The returned values can be used in a PID controller as "setpoint" vs a wheel encoder's feedback expressed in RPM.
  - linear_x : target linear speed of the robot in x axis (forward or reverse) expressed in m/s.
  - linear_y : target linear speed of the robot in y axis (strafing left or strafing right for mecanum drive) expressed in m/s.
  - angular_z : target angular speed of the robot in z axis (rotating CCW or CW) rad/sec.

#### 3. output getPWM(float linear_x, float linear_y, float angular_z)
The same output as getRPM() function converted to a PWM value. The returned values can be used to drive motor drivers using the PWM signals.

#### 4. velocities getVelocities(int motor1, int motor2)
This is the inverse of getRPM(). Returns linear velocities in x and y axis, and angular velocity in z axis given two measured RPMs on each motor of a 2WD robot. The returned values can be used to calculate the distance traveled in a specific axis - where distance traveled is the product of the change in velocity and change in time.
  - motor1: left motor's measured RPM
  - motor2: right motor's measured RPM

*each motor's RPM value must be signed. + to signify forward direction and - to signify reverse direction of the motor.

#### 5. velocities getVelocities(int motor1, int motor2, int motor3, int motor4)
The same output as No.4 but requires 4 measured RPMs on each motor of a 4WD robot. This can be used for both 4 wheeled differential drive and mecanum drive robots.
  - motor1: front left motor's measured RPM
  - motor2: front right motor's measured RPM
  - motor3: front right motor's measured RPM
  - motor4: front right motor's measured RPM

*each motor's RPM value must be signed. + to signify forward direction and - to signify reverse direction of the motor.

## Data structures
#### 1. output
  Struct returned by getRPM() and getPWM used to store PWM or RPM values.
  ```
  struct output{
    int motor1;
    int motor2;
    int motor3;
    int motor4;
  };
  ```
* each returned motor RPM or PWM value is signed to signify the motor's direction. + forward direction ; - reverse direction.  

#### 2. velocities
  Struct returned by getVelocities() used to store linear velocities in x and y axis, and angular velocity in z axis (right hand rule).
  ```
  struct velocities
  {
    float linear_x;
    float linear_y;
    float angular_z;
  };
  ```
  * linear_x and linear_y are expressed in m/s. angular_z is expressed in rad/s
