#include "Arduino.h"
#include "Motor.h"

Motor::Motor(int pwm_pin, int motor_pinA, int motor_pinB)
{
  motor_driver_ = L298;

  pinMode(pwm_pin, OUTPUT);
  pinMode(motor_pinA, OUTPUT);
  pinMode(motor_pinB, OUTPUT);

  pwm_pin_ = pwm_pin;
  motor_pinA_ = motor_pinA;
  motor_pinB_ = motor_pinB;
}

Motor::Motor(int motor_pinA, int motor_pinB)
{
  motor_driver_ = BTS7960;

  pinMode(motor_pinA, OUTPUT);
  pinMode(motor_pinB, OUTPUT);

  motor_pinA_ = motor_pinA;
  motor_pinB_ = motor_pinB;
}

void Motor::updateSpeed(long encoder_ticks)
{
  //this function calculates the motor's RPM based on encoder ticks and delta time
  unsigned long current_time = millis();
  unsigned long dt = current_time - prev_update_time_;

  //convert the time from milliseconds to minutes
  double dtm = (double)dt / 60000;
  double delta_ticks = encoder_ticks - prev_encoder_ticks_;

  //calculate wheel's speed (RPM)
  rpm = (delta_ticks / counts_per_rev_) / dtm;

  prev_update_time_ = current_time;
  prev_encoder_ticks_ = encoder_ticks;
}

void Motor::spin(int pwm)
{
  if(motor_driver_ == L298)
  {
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
  }
  else if(motor_driver_ == BTS7960)
  {
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
  }
}
