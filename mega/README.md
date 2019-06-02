# Arduino MEGA 2560 port

This is unofficial port for Arduino MEGA 2560.

## Limitations

1. You can use only 2WD movement by Arduino MEGA 2560
2. Sometimes it can be ["runaway robot"](https://groups.google.com/d/msg/linorobot/Twc49Mdzais/CDNbNOxZAwAJ) condition
3. Can be used only L298 like motor controllers (you can try to use motor shield, but then change pins)

## Changes

### Pins

Pins in the config file was changed.

```
#define MOTOR1_PWM 4
#define MOTOR1_IN_A 44
#define MOTOR1_IN_B 45

#define MOTOR2_PWM 13
#define MOTOR2_IN_A 46
#define MOTOR2_IN_B 47
```

1. There are used 4th and 13th pins of PWM because they have highest frequency.
2. But you can use different pins!

```
#define PWM_MAX 255
#define PWM_MIN -PWM_MAX
```

1. PWM_MAX value was hardcoded, because of [this issue](https://github.com/linorobot/linorobot/issues/27)

### PWM messages

Output of two PWM values were added, it is useful to tests with Arduino.

```
void printPWM(int m, long val)
{
    char buffer[50];

    sprintf (buffer, "PWM to m%d value is  : %ld", m, val);
    nh.loginfo(buffer);
}
```

This function is called in the main loop before publishing.

```

    printPWM(1, motor1_pid.compute(req_rpm.motor1, current_rpm1));
    printPWM(2, motor2_pid.compute(req_rpm.motor2, current_rpm2));

```

## Author
This changes were made by unofficial person, there may be errors. Team of linorobot is not responsible for this port.
