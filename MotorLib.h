//The original base lib can be found at https://github.com/zaidpirwani/DiffMotorVelocityControl

#ifndef MotorLib_h
#define MotorLib_h

#if defined (ARDUINO) && ARDUINO >= 100
  #include <Arduino.h>
#else
  #include <WProgram.h>
  #include <pins_arduino.h>
#endif

#if defined (__AVR__)
  #include <avr/io.h>
#endif

#define FORWARD   1
#define BACKWARD  2
#define BRAKE     3
#define RELEASE   4

class MotorLib {
  public:
    MotorLib(uint8_t ch1_pin, uint8_t ch2_pin, uint8_t pwm_pin);
    void setPWM(uint8_t pwm);
    unsigned int getPWM();
    void setDir(uint8_t dir);
    unsigned int getDir();
  private:
    uint8_t _ch1_pin;
    uint8_t _ch2_pin;
    uint8_t _pwm_pin;
    uint8_t _pwm;
    uint8_t _dir;
};

#endif

