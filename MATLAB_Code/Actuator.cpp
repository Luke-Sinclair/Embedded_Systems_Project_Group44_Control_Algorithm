#include "Actuator.h"
#include "Arduino.h"
Actuator::Actuator(int pin){
      _pin = pin;
      pinMode(_pin,OUTPUT);
      }
void Actuator::updateServo(int pWidth){
     digitalWrite(_pin,HIGH);
     delayMicroseconds(pWidth);
     digitalWrite(_pin,LOW);
    // delay(20);
    }

void Actuator::setAngle(int angle){
updateServo((10*angle +900));
}
