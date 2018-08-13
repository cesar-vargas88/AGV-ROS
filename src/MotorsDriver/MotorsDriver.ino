#include "Car4W.h"
#include "MotorCD.h"
#include <ros.h>
#include <std_msgs/Int32.h>

Car4W Car4WWarehouse( 12 , 11 , 13 ,      // Motor Front Left
                       9 , 10 ,  8 ,      // Motor Front Right
                       4 ,  3 ,  2 ,      // Motor Rear Left 
                       5 ,  6 ,  7 );     // Motor Rear Right            

ros::NodeHandle  nh;

int nPID      = 0;
int nSpeed    = 255;
int nSwitch   = 0;

void MotorsDriver_Callback( const std_msgs::Int32& msg)
{
  nPID = msg.data;
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led 
  
  if(nSwitch == 1)
  {
      Car4WWarehouse.Forward(nSpeed, nSpeed, nSpeed, nSpeed);
      nSwitch = 0;
  }    
  else
  {
     Car4WWarehouse.Stop();  
     nSwitch = 1;
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

void loop()
{ 
  nh.spinOnce();
  delay(2000);
}
