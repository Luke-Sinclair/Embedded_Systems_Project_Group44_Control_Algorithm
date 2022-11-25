#include "Sonar.h"
#include "Arduino.h"

Sonar::Sonar(int signalPin){
            _signalPin=signalPin;
          };


                
float Sonar::getSonarDataCm(){
  pinMode(_signalPin,OUTPUT);
  digitalWrite(_signalPin,LOW);
  delayMicroseconds(3);
  digitalWrite(_signalPin,HIGH);
  delayMicroseconds(5);
  digitalWrite(_signalPin,LOW);
  pinMode(_signalPin,INPUT);
  pulseDuration = pulseIn(_signalPin, HIGH, 3000);
  float pulseFloat = (float) pulseDuration;
  lastDistance= pulseFloat *0.01715;
  return lastDistance;
  }
