#include <Arduino.h>
#include <ReflectanceSensor.h>
#include <MotorDriver.h>
#include "Sonar.h"
#include "actuator.h"
#include "Encoder.h"

class PID{
  private:
    float previousError;
    float PID_I;
    float PID_D;
    float PID_P;
    
    float kP;
    float kI;
    float kD;
    float PID_Val;
    float error;
  public:
    float getError(){return error;}
    float getPID(){return PID_Val;};
    
    PID(){
      error=0;
      PID_I=0;
      }
    void setConst(float p, float i, float d){
      kP = p;
      kI = i;
      kD = d;
      }      
    void updatePid(float dt,float setPoint, float processVariable){
      error = setPoint-processVariable;
      PID_P = kP*error;
      PID_I = PID_I + kI*error*dt;
      PID_D = kD*(error-previousError)/dt;
      PID_Val = PID_P + PID_I + PID_D;
      previousError = error;
      }
  };


ReflectanceSensor sensor;

Encoder enc(34,36,35,39);
float angle=0;
float delta_angle=0;
float newAngle=0;
float distance=0;
float velocity;

PID rCtrl;
PID lCtrl;
PID linepid;
PID anglePID;
bool turnAround = false;

MotorDriver Mr;
MotorDriver Ml;
Actuator actuator(5);
Sonar sonar(14);
uint8_t SensorCount = 6; //# of reflectance sensor

//here for sensorpins
uint8_t SensorPins[6] = {32,25,27,22,23,21}; //Sensor 1 leftmost in array
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^



uint32_t Timeout = 2500; //reflect timeout(us)

//right Motor configuration variables
int motorR_type = 2;
int motR_pins[3] = {4,15,18};
int motR_sign = -1;
float gear_ratio_r = 34;

// Left Motor configuration variables
int motorL_type = 2;
int motL_pins[3] = {2,12,0};
int motL_sign = 1;
float gear_ratio_l = 34;


void setup()
{

  rCtrl.setConst(0.34,0,0);
  lCtrl.setConst(0.34,0,0);
  linepid.setConst(0.195,0.000005,0.0005);
  anglePID.setConst(0.005,0,0.001);
  
  // configure the sensors
  sensor.SetSensorPins(SensorPins, SensorCount);
  sensor.SetTimeout (Timeout);
  
  Mr.DriverSetup(motR_pins[0],0, motR_pins[1], motR_pins[2]);
  Mr.SetBaseFreq(5000);
  Mr.SetMotorType(motorR_type);
  Mr.SetSign(motR_sign);
  
  Ml.DriverSetup(motL_pins[0],1, motL_pins[1], motL_pins[2]);
  Ml.SetBaseFreq(5000);
  Ml.SetMotorType(motorL_type);
  Ml.SetSign(motL_sign);

  Mr.MotorWrite(0);
  Ml.MotorWrite(0);

  Serial.begin(115200);
}

volatile int Encoder::current_pulseL=0;
volatile int Encoder::current_pulseR=0;
float turnAroundCounter=0;
float time_control = 0;
float lastave;
float lwheel=0;
float rwheel=0;
int loopcounter=0;
float offLineCounter=0;
void loop()
{
    enc.updateValues(lCtrl.getPID(),rCtrl.getPID());
    delta_angle = (enc.getSpeedR() + enc.getSpeedL())/0.18;
    velocity = (abs(enc.getSpeedR()) + abs(enc.getSpeedL()))/2;
    loopcounter++;
    float current_time = millis();
    float dt =( current_time-time_control)/1000;
    time_control = current_time;
    angle = angle + delta_angle*dt;
    
    int sensorsOffLine=0;
    float reflectance_n[7];
    sensor.ReadSensor();
    float sum =0;
    float ave =0;
    
    float weightArray [6] = {-3,-2,-1,1,2,3};
    for (int i=0;i<sensor.GetSensorCount();i++)
    {
            reflectance_n[i] = sensor.GetSensorValues(i);
            //
            //HERE FOR THE LIGHT SENSOR CALLIBRATION
            if(reflectance_n[i]<=600){
            //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
            //ABOVE LINE FOR LIGHT SEnSOR CALLIBRATION
               reflectance_n[i] =1;
              }else{
              reflectance_n[i]=0;  
                }
            if (reflectance_n[i]>0 ){ //count number of sensors being activated by the line
                sensorsOffLine = sensorsOffLine+1; 
            }
            reflectance_n[i] = reflectance_n[i]*  weightArray[i];;
            //*  weightArray[i];
            sum += reflectance_n[i]; //add up all the error values for sensors on the line
 
    }
     if (sensorsOffLine > 0){
                  
      rCtrl.setConst(0.34,0,0);
      lCtrl.setConst(0.34,0,0);
      ave = sum/sensorsOffLine;
      lastave=ave;
      offLineCounter=0;
    }else{
     ave = lastave;
     offLineCounter=offLineCounter+dt;
    }
    
    /*
    Serial.print(" Error ");
    
    Serial.print(ave);
   
    Serial.println(" ");

    for(int i=0;i<sensor.GetSensorCount();i++){
      Serial.print(i);
      Serial.print(" : ");
      Serial.print(reflectance_n[i]);
      }
     Serial.println("  ");
     */
    linepid.updatePid(dt,0,ave);
    if(!turnAround){
        lwheel= -0.5 - linepid.getPID() ;
        rwheel = 0.5 - linepid.getPID();
        }
    anglePID.updatePid(dt, newAngle, angle);
    if(turnAround &&offLineCounter<0.4){
          turnAroundCounter = turnAroundCounter+dt;
          if(turnAroundCounter<0.6){
                         
              rCtrl.setConst(0.01,0,0);
              lCtrl.setConst(0.01,0,0);
              lwheel= 2.5;
              rwheel=-2.5;
              
            }  
           else{
          if(turnAroundCounter<1.5){
          
          anglePID.updatePid(dt, newAngle, angle);         
          rCtrl.setConst(0.15,0,0.0001);
          lCtrl.setConst(0.15,0,0.0001);
          lwheel= 0.65;
          rwheel=0.65;
          if(linepid.getError()<1 && sensorsOffLine>0 && turnAroundCounter >0.7 ){
            
            turnAround = false;
            }
          }else{
                      
            turnAround = false;
            Serial.print("Out");
            }
           }
    }


    if(offLineCounter>0.35 && !turnAround){
      turnAroundCounter=0;
      rCtrl.setConst(0.2,0,0.000);
      lCtrl.setConst(0.2,0,0.000); 
      rwheel=0;
      lwheel=0;
      }
    
    rCtrl.updatePid(dt,rwheel*9, enc.getSpeedR());
    lCtrl.updatePid(dt,lwheel*9, enc.getSpeedL()); 

    
    Mr.MotorWrite(rCtrl.getPID());
    Ml.MotorWrite(lCtrl.getPID());
    if((sonar.getSonarDataCm()<11 && sonar.lastDistance !=0) && !turnAround){
      turnAround = true;
      }
    
    delay(12);
}
