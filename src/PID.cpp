#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "Minotauro/Trigger.h"
#include "Minotauro/PIDTick.h"

class PID
{
    public:

        PID(float kP, float kI, float kD, float tiempoMuestreo, int speed)
        {
            KP  = kP;
            KI  = kI;
            KD  = kD;
            TiempoMuestreo = tiempoMuestreo;

            Error           = 0;
            LastError       = 0;
            Proportional    = 0;
            Integral        = 0;
            Derivative      = 0;
        }

        ~PID()
		{
		}

        int GetOutput(int error)
        {
            Error        =  error;
            Proportional =  Error;
            Integral    +=  Error * TiempoMuestreo;
            Derivative   = (Error - LastError) / TiempoMuestreo;

            LastError = Error;

            return (int) (KP * Error) + (KI * Integral) + (KD * Derivative);
        }

    private:

        float KP;
        float KI;
        float KD;

    	float Error;
        float LastError;

        float Proportional;
        float Integral;
        float Derivative;

        float TiempoMuestreo;
};

ros::ServiceClient 	CameraTrigger_Client; 
ros::Subscriber 	PIDTick_Subscriber;
ros::Publisher 		MotorsDriver_Publisher;

std_msgs::Int32 PIDOutput;
Minotauro::Trigger srv;

int PIDTick = 0;

PID  pid(5, 0.5, 2, 0.1, 0.1);

void PIDTick_Callback(const std_msgs::Int32::ConstPtr& msg)
{
	PIDTick = msg->data;	
	ROS_INFO("PIDTick received: %d", msg->data);

	if(PIDTick == 1)
	{		
		PIDOutput.data = PIDTick;
		if (CameraTrigger_Client.call(srv))
		{
			PIDOutput.data = pid.GetOutput(srv.response.CameraError);	
			ROS_INFO("Trgger: %d , %d", srv.response.Success, srv.response.CameraError);					
		}
 		else
			ROS_ERROR("Failed to call Camera_Service");
	}	
	else
		PIDOutput.data = PIDTick;
	
	ROS_INFO("PIDOutput: %d", PIDOutput.data);
	MotorsDriver_Publisher.publish(PIDOutput);	
}

int main(int argc, char **argv)
{
  	ros::init(argc, argv, "PID_Node");
  	ros::NodeHandle n;

  	CameraTrigger_Client = n.serviceClient<Minotauro::Trigger>("CameraTrigger_Service");

  	PIDTick_Subscriber = n.subscribe("PIDTick_Topic", 1000, PIDTick_Callback);
  	ros::spinOnce();

  	MotorsDriver_Publisher = n.advertise<std_msgs::Int32>("MotorsDriver_Topic", 1000);	
  	ros::Rate loop_rate(2);

	while(true)
	{
		/*if(PIDTick == 1)
		{		
			PIDOutput.data = PIDTick;
			if (CameraTrigger_Client.call(srv))
			{
				PIDOutput.data = pid.GetOutput(srv.response.CameraError);	
				ROS_INFO("Trgger: %d , %d", srv.response.Success, srv.response.CameraError);					
			}
 			else
				ROS_ERROR("Failed to call Camera_Service");
		}	
		else
			PIDOutput.data = PIDTick;
		
		ROS_INFO("PIDOutput: %d", PIDOutput.data);
		MotorsDriver_Publisher.publish(PIDOutput);	

*/
		ros::spinOnce();
	 	loop_rate.sleep();	
	}

  	return 0;
}
