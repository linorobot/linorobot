
# Lino Robot Troubleshooting Guide

Try to re-calibrate IMU.

Check the odomentry.

Check the power supply , continuity of the circuit and do the below.

1) Check all the motors are working by directly connecting the power supply.
2) Check whether the teensy is giving the PWM signals or not by connecting a LED to it or by using a multimeter .
3) Check whether the encoder value increse and decrease by rotating the wheel back and forth.(run minimal.launch file)

If all the above are working and still the wheel is spinning in random direction and not following the key commands , try the below points.


Check all the encoder value are producing zero or in range 0 to 100 when launching minimal.launch file.

Check that for single forward and reverse press in keyboard the value change is around 400(in our case)

Disconnecting the encoder pins A and B will not reset the value to zero it’ll stall the value and it won’t affect the wheel spin.

Swapping the PWM pins from the corresponding motor driver will make the wheels to spin in the correct direction.

Do not try to change the PWM pins configuration from the ‘config’ file use the default pins and also make sure that they are PWM.

If the wheels are still spinning continuously try swapping the motor’s pins (+ve and -ve pins) by placing the PWM pins in the motor driver in it’s corresponding pins.

If you are getting “IMU not connected” then find the device ID and add the device ID  to this file teensy/firmware/lib/imu/MPU9250.cpp (9250 in our case) add Device Id(0x39).

Make sure that the motor are connected to the common ground.

If the robot is working and moving slowly in SLAM navigation try to reduce **inflation_radius** parameter from the file "linorobot/param/navigation/costmap_common_params.yaml"
