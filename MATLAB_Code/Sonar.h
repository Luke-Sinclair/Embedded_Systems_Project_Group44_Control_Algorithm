#ifndef Sonar_H
#define Sonar_H
#include <Arduino.h>
class Sonar{
      private:
        int _signalPin;
        unsigned long pulseDuration;
		
      public:
        float lastDistance;
        Sonar(int signalPin);
        float getSonarDataCm();
  };
#endif