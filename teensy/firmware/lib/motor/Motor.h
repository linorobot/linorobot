#ifndef MOTOR_H
#define MOTOR_H

#include <Servo.h> 
#include <Arduino.h>

class Motor
{
    public:
        enum driver {L298, BTS7960, ESC};
        Motor(driver motor_driver, int counts_per_rev, int pwm_pin, int motor_pinA, int motor_pinB);
        void updateSpeed(long encoder_ticks);
        void spin(int pwm);
        int getRPM();

    private:
        Servo motor_;
        int rpm_;
        unsigned long prev_update_time_;
        long prev_encoder_ticks_;
        driver motor_driver_;
        int counts_per_rev_;
        int pwm_pin_;
        int motor_pinA_;
        int motor_pinB_;
};

#endif
