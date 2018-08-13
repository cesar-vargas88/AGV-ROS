#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "Minotauro/Trigger.h"

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

bool PIDTick = false;

void PIDTick_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	PIDTick = msg->data;	
	//ROS_INFO("PIDTick received: %d", msg->data);
}

int main(int argc, char **argv)
{
  	ros::init(argc, argv, "PID_Node");
  	ros::NodeHandle n;

  	ros::ServiceClient CameraTrigger_Client = n.serviceClient<Minotauro::Trigger>("CameraTrigger_Service");

  	ros::Subscriber PIDTick_Subscriber = n.subscribe("PIDTick_Topic", 1000, PIDTick_Callback);
  	ros::spinOnce();

  	ros::Publisher MotorsDriver_Publisher = n.advertise<std_msgs::Int32>("MotorsDriver_Topic", 1000);	
  	ros::Rate loop_rate(100);

  	PID  pid(5, 0.5, 2, 0.1, 0.1);

  	std_msgs::Int32 msg;

	Minotauro::Trigger srv;

	while(true)
	{
		if(PIDTick)
		{		
			if (CameraTrigger_Client.call(srv))
			{
				msg.data =  pid.GetOutput(srv.response.CameraError);	
				ROS_INFO("Trgger response: %d , %d , PID output: %d", srv.response.Success, srv.response.CameraError, msg.data);
				MotorsDriver_Publisher.publish(msg);	
				
			}
 			else
				ROS_ERROR("Failed to call Camera_Service");

			PIDTick = false;	
		}	

		ros::spinOnce();
	 	loop_rate.sleep();	
	}

  	return 0;
}
