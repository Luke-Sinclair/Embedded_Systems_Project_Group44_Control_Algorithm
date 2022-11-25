#ifndef ACTUATOR_H
#define ACTUATOR_H
#include <Arduino.h>

class Actuator{
  private:
    int _pin;
     void updateServo(int pWidth);
  public:
    Actuator(int pin);
      //run setAngle In main loop, between 0 and 120 degrees
      void setAngle(int angle);
  };
#endif