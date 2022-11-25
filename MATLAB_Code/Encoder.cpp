#include "Encoder.h"
#include "Arduino.h"
    
    
    void Encoder::rchannelA(){
      current_pulseR++;
    }
    
    void Encoder::rchannelB(){
      current_pulseR++;
      
    }
    
    void  Encoder::lchannelA(){
      current_pulseL++;
    }
    
    void  Encoder::lchannelB(){
      current_pulseL++;
    }
    
   
    Encoder::Encoder(int rPin1,int rPin2, int lPin1, int lPin2){
      wheel_diametre =34;
      gear_ratio=34;
      current_pulseL=0;
      current_pulseR=0;
      int _lPin1 =lPin1;
      int _lPin2=lPin2;
      int _rPin1=rPin1;
      int _rPin2=rPin2;   
      pinMode(_rPin1, INPUT_PULLUP);
      pinMode(_rPin2, INPUT_PULLUP);
      attachInterrupt(digitalPinToInterrupt(_rPin1),rchannelA, RISING);
      
      attachInterrupt(digitalPinToInterrupt(_rPin2),rchannelB, RISING);
      pinMode(_lPin1, INPUT_PULLUP);
      pinMode(_lPin2, INPUT_PULLUP);
      attachInterrupt(digitalPinToInterrupt(_lPin1),lchannelA, RISING);
      attachInterrupt(digitalPinToInterrupt(_lPin2),lchannelB, RISING);

      };
      
      float Encoder::getSpeedL(){
        return speedL;
        }
      float Encoder::getSpeedR(){
        return speedR;
        }  
      void Encoder::updateValues(float leftpwm, float rightpwm){

        
        current_timeR = millis();
        current_timeL = millis();
        if ((current_timeR-start_timeR)>10){
          speedR = 0.002857*( ((current_pulseR-previous_pulseR)*wheel_diametre*gear_ratio)/((current_timeR-start_timeR)) );
          start_timeR = current_timeR;
          previous_pulseR = current_pulseR;
        }
        
        if ((current_timeL-start_timeL)>10){
          speedL = 0.002857*(((current_pulseL-previous_pulseL)*wheel_diametre*gear_ratio)/((current_timeL-start_timeL)));
          start_timeL = current_timeL;
          previous_pulseL = current_pulseL;
        }


        if(leftpwm<0){
          speedL = abs(speedL)*-1;
          }
         if(rightpwm<0){
          speedR = abs(speedR)*-1;
          }

      }
