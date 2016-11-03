#include "MotorLib.h"

 
MotorLib::MotorLib(uint8_t ch1_pin, uint8_t ch2_pin, uint8_t pwm_pin) {
  _ch1_pin = ch1_pin;
  _ch2_pin = ch2_pin;
  _pwm_pin = pwm_pin;
  pinMode(ch1_pin, OUTPUT);
  pinMode(ch2_pin, OUTPUT);
  pinMode(pwm_pin, OUTPUT);
  setDir(FORWARD);
}

void MotorLib::setPWM(uint8_t pwm) {
  _pwm = pwm;
  analogWrite(_pwm_pin,_pwm);
}

unsigned int MotorLib::getPWM() {
  return(_pwm);
}

void MotorLib::setDir(uint8_t dir) {
  if(_dir!=dir){
    _dir = dir;   
    switch(_dir){
      case FORWARD:
        digitalWrite(_ch1_pin, HIGH);
        digitalWrite(_ch2_pin, LOW);
        break;
      case BACKWARD:
        digitalWrite(_ch1_pin, LOW);
        digitalWrite(_ch2_pin, HIGH);
        break;
      case BRAKE:
        digitalWrite(_ch1_pin, HIGH);
        digitalWrite(_ch2_pin, HIGH);
        break;
      case RELEASE:
        digitalWrite(_ch1_pin, LOW);
        digitalWrite(_ch2_pin, LOW);
        break;
    }
  }
}

unsigned int MotorLib::getDir() {
  return(_dir);
}
