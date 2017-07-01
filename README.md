# linorobot
Linorobot is a suite of Open Source ROS compatible robots for that aims to provide students, developers, and researchers a low-cost platform in creating new exciting applications on top of ROS.

## Tutorial

You can read the full tutorial here: http://linorobot.org

## Multiplatform
Supports multiple types of robot base:
- 2WD
- 4WD
- Ackermann Steering 
- Mecanum drive

![alt text](https://github.com/linorobot/lino_docs/blob/master/imgs/family.png?raw=true)


## Hardware
Fabricate your own Teensy 3.1/3.2 shield,

![alt text](https://github.com/linorobot/lino_docs/blob/master/imgs/shield.JPG?raw=true)![alt text](https://github.com/linorobot/lino_docs/blob/master/imgs/shield2.JPG?raw=true)

or wire it on your own. Wiring diagrams are also provided.
![alt text](https://github.com/linorobot/lino_docs/blob/master/imgs/schematicsfamilyphoto.png?raw=true)

#### Supported IMUs:
- GY-85
- MPU6050 (to be released)

#### Supported Motor Drivers:
- L298
- BTS7960   
**This should easily work with other motor drivers as long as the pins are compatible.

#### Supported ROS Compatible Sensors:
- XV11 Lidar
- RPLidar
- Intel RealSense R200 (to be released)
- Kinect (to be released)

#### Tested on Linux compatible ARM dev boards:    
- Raspberry Pi 3   
- Jetson TK1   
- Jetson TX1   
- Odroid XU4   
- Radxa Rock Pro   
**Technically this should also work with any ARM dev board at least (1GB RAM) that runs Ubuntu Trusty or Xenial.

## Installation
![alt text](https://github.com/linorobot/lino_docs/blob/master/imgs/installationshot.png?raw=true)

```
$ git clone https://github.com/linorobot/lino_install && cd lino_install
$ ./install
```

## Firmware
Flexible and configurable components.
linorobot_ws/teensy/firmware/lib/config/lino_base_config.h

#### Robot base configuration:
```
//uncomment the base you're building
#define LINO_BASE DIFF_2WD
// #define LINO_BASE DIFF_4WD
// #define LINO_BASE ACKERMANN
// #define LINO_BASE HOLO_4W
```

#### IMU configuration:
```
//uncomment the IMU you're using
#define GY85_IMU
// #define MP6050_IMU (not supported yet)
```

#### Motor driver configuration:
```
//uncomment the motor driver you're using
#define MOTOR_DRIVER L298
// #define MOTOR_DRIVER BTS7960
```

#### Motor configuration:
```
//define your robot' specs here
#define MAX_RPM 330 // motor's maximum RPM
#define COUNTS_PER_REV 1550 // wheel encoder's no of ticks per rev
#define WHEEL_DIAMETER 0.10 // wheel's diameter in meters
#define PWM_BITS 8 // PWM Resolution of the microcontroller
#define BASE_WIDTH 0.235 // width of the plate you are using
```

## Creating a Map
![alt text](https://github.com/linorobot/lino_docs/blob/master/imgs/slam.png?raw=true)

#### Launch base driver:
```
$ roslaunch linorobot bringup.launch
```

#### Launch mapping packages:
```
roslaunch linorobot slam.launch
```

## Autonomous Navigation
[![IMAGE ALT TEXT](http://img.youtube.com/vi/aqzMq-jMd-c/maxresdefault.jpg)](https://www.youtube.com/embed/aqzMq-jMd-c "Linorobot Autonomous Navigation")

#### Launch base driver:
```
$ roslaunch linorobot bringup.launch
```

#### Launch navigation packages:
```
roslaunch linorobot navigate.launch
```