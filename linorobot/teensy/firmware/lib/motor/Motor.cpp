#include "Motor.h"

Controller::Controller(driver motor_driver, int pwm_pin, int motor_pinA, int motor_pinB):
    motor_driver_(motor_driver),
    pwm_pin_(pwm_pin),
    motor_pinA_(motor_pinA),
    motor_pinB_(motor_pinB)
{
    switch (motor_driver)
    {
        case L298:
            pinMode(pwm_pin_, OUTPUT);
            pinMode(motor_pinA_, OUTPUT);
            pinMode(motor_pinB_, OUTPUT);

            //ensure that the motor is in neutral state during bootup
            analogWrite(pwm_pin_, abs(0));

            break;

        case BTS7960:
            pinMode(motor_pinA_, OUTPUT);
            pinMode(motor_pinB_, OUTPUT);

            //ensure that the motor is in neutral state during bootup
            analogWrite(motor_pinB_, 0);
            analogWrite(motor_pinA_, 0);

            break;

        case ESC:
            motor_.attach(motor_pinA_);

            //ensure that the motor is in neutral state during bootup
            motor_.writeMicroseconds(1500);

            break;
    }
}

void Controller::spin(int pwm)
{
    switch (motor_driver_)
    {
        case L298:
            if(pwm > 0)
            {
                digitalWrite(motor_pinA_, HIGH);
                digitalWrite(motor_pinB_, LOW);
            }
            else if(pwm < 0)
            {
                digitalWrite(motor_pinA_, LOW);
                digitalWrite(motor_pinB_, HIGH);
            }
            analogWrite(pwm_pin_, abs(pwm));

            break;

        case BTS7960:
            if (pwm > 0)
            {
                analogWrite(motor_pinA_, 0);
                analogWrite(motor_pinB_, abs(pwm));
            }
            else if (pwm < 0)
            {
                analogWrite(motor_pinB_, 0);
                analogWrite(motor_pinA_, abs(pwm));
            }
            else
            {
                analogWrite(motor_pinB_, 0);
                analogWrite(motor_pinA_, 0);
            }

            break;
        
        case ESC:
            motor_.writeMicroseconds(1500 + pwm);

            break;
    }
}
