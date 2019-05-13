#ifndef MOTOR_H
#define MOTOR_H

#include <Servo.h> 
#include <Arduino.h>

#ifndef NEUTRAL_BAND_WAIT_MS_PER_PWM
#define NEUTRAL_BAND_WAIT_MS_PER_PWM 3
#endif

class Controller
{
    public:
        enum driver {L298, BTS7960, ESC};
        Controller(driver motor_driver, int pwm_pin, int motor_pinA, int motor_pinB);
        int spin(int pwm);
		int latest_spin_applied() { return this->prev_pwm_;};

    private:
        Servo motor_;
        driver motor_driver_;
        int pwm_pin_;
        int motor_pinA_;
        int motor_pinB_;
		int prev_pwm_;
};

#endif
