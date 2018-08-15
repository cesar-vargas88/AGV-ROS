#include "Car4W.h"
#include "MotorCD.h"
#include <ros.h>
#include <std_msgs/Int32.h>
 
Car4W Car4WWarehouse(  9 , 10 ,  8 ,      // Motor Front Left
                      12 , 11 , 13 ,      // Motor Front Right
                       4 ,  3 ,  2 ,      // Motor Rear Left 
                       5 ,  6 ,  7 );     // Motor Rear Right    
                       
ros::NodeHandle  nh;

int nSpeed    = 255;

int SpeedMotorFrontLeft  = 0;
int SpeedMotorRearLeft   = 0;
int SpeedMotorFrontRight = 0;
int SpeedMotorRearRight  = 0;

void MotorsDriver_Callback( const std_msgs::Int32& msg)
{
  if(msg.data == 0)
  {
    Car4WWarehouse.Stop(); 
    digitalWrite(13, LOW);
  }
  else
  {
    SpeedMotorFrontLeft  = nSpeed + msg.data;
    SpeedMotorRearLeft   = nSpeed + msg.data;
    SpeedMotorFrontRight = nSpeed - msg.data;
    SpeedMotorRearRight  = nSpeed - msg.data;
    
    constrain(SpeedMotorFrontLeft , 0, 255);
    constrain(SpeedMotorRearLeft  , 0, 255);
    constrain(SpeedMotorFrontRight, 0, 255);
    constrain(SpeedMotorRearRight , 0, 255);

    Car4WWarehouse.Forward(SpeedMotorRearLeft, SpeedMotorRearRight, SpeedMotorFrontLeft, SpeedMotorFrontRight);
    
    digitalWrite(13, HIGH);//-digitalRead(13));   // blink the led
  }
}

ros::Subscriber<std_msgs::Int32> MotorsDriver_Subscriber("MotorsDriver_Topic", MotorsDriver_Callback);

void setup()
{
  Car4WWarehouse.Setup();
  pinMode(13, OUTPUT);
  
  nh.initNode();
  nh.subscribe(MotorsDriver_Subscriber);
}

int Speed = 255;


int nA   = 5;
int nB   = 6;
int nPWM = 7;

void loop()
{ 
  nh.spinOnce();
  // Forward
  /*digitalWrite(nA, HIGH);
  digitalWrite(nB, LOW);
  analogWrite(nPWM, Speed);
  
  delay(5000);
  
  digitalWrite(nA, LOW);
  digitalWrite(nB, LOW);
  analogWrite(nPWM, Speed);
  
  delay(1000);
  
  digitalWrite(nA, LOW);
  digitalWrite(nB, HIGH);
  analogWrite(nPWM, Speed);
  
  delay(5000);*/
  delay(100);
}
