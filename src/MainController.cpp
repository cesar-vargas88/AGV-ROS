#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "Minotauro/HMI.h"

std_msgs::Int32 snPIDTick;

bool HMI_Input_Callback(Minotauro::HMI::Request  &req, Minotauro::HMI::Response &res)
{
	switch(req.channel)
	{
		case 0:
			snPIDTick.data = req.message;			
			break;
		case 1:
			break;
		case 2:
			break;
		case 3:
			break;
		case 4:
			break;
		case 5:
			break;
		case 6:
			break;
		case 7:
			break;
		case 8:
			break;
		case 9:
			break;
		case 10:
			break;		
	}

	ROS_INFO("HMI input received: %d , %d", (int) req.channel, (int) req.message);

  	return true;
}

void RFID_Callback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("RFID: [%s]", msg->data.c_str());
}

void LiDAR_Callback(const std_msgs::Int32::ConstPtr& msg)
{
	ROS_INFO("LiDAR: [%d]", msg->data);
}

void HMI_Callback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("HMI: [%s]", msg->data.c_str());
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "MinotauroController_Node");
	ros::NodeHandle n;

	ros::Subscriber RFID_Subscriber 	= n.subscribe("RFID_Topic" , 1000, RFID_Callback);
	ros::Subscriber LiDAR_Subscriber 	= n.subscribe("LiDAR_Topic", 1000, RFID_Callback);
	ros::Subscriber HMI_Subscriber 		= n.subscribe("HMI_Topic"  , 1000, RFID_Callback);

	ros::spinOnce();

	ros::ServiceServer HMI_Input_Server = n.advertiseService("HMI_Input_Service", HMI_Input_Callback);

	ros::Publisher PIDTick_Publisher 	= n.advertise<std_msgs::Int32>("PIDTick_Topic", 1000);

	ros::Rate loop_rate(20);

	while (ros::ok())
	{
		ROS_INFO("PIDTick: %d", (int) snPIDTick.data);
		PIDTick_Publisher.publish(snPIDTick);
		
	 	ros::spinOnce();
	 	loop_rate.sleep();
	 }

	return 0;
}

