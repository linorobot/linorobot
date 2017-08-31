#ifndef MOTOR_H
#define MOTOR_H

#include "Arduino.h"

class Motor
{
    public:
        enum driver {L298, BTS7960};
        Motor(driver motor_driver, int counts_per_rev, int pwm_pin, int motor_pinA, int motor_pinB);
        //Motor(int motor_pinA, int motor_pinB);
        void updateSpeed(long encoder_ticks);
        void spin(int pwm);
        int getRPM();

    private:
        int rpm_;
        int counts_per_rev_;
        driver motor_driver_;
        long prev_encoder_ticks_;
        unsigned long prev_update_time_;
        int pwm_pin_;
        int motor_pinA_;
        int motor_pinB_;
};

#endif
