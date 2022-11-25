#ifndef ENCODER.h
#define ENCODER.h
#include <Arduino.h>

class Encoder{
  static volatile int current_pulseL;
  static volatile int current_pulseR;
  private:
    
  
    int _lPin1;
    int _lPin2;
    int _rPin1;
    int _rPin2; 
    float gear_ratio=34;
    float wheel_diametre=0.101;
    float encoderR_ticks =48;
    unsigned long current_timeR;
    unsigned long start_timeR= millis();
    int previous_pulseR;
    float speedR;
    //left Encoder configuration variables
    float encoderL_ticks=48;
    unsigned long current_timeL;
    unsigned long start_timeL= millis();
    int previous_pulseL;
    float speedL;
    static void rchannelA();
    static void rchannelB();
    static void lchannelA();
    static void lchannelB();
    
  public:
    Encoder(int rPin1,int rPin2, int lPin1, int lPin2);
      float getSpeedL();
      float getSpeedR();
      void updateValues(float leftpwm, float rightpwm);
 
  };

#endif
