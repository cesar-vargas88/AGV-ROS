#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "Minotauro/HMI.h"

int HMI_Input[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

bool HMI_Input_Callback(Minotauro::HMI::Request  &req, Minotauro::HMI::Response &res)
{
	HMI_Input[req.channel] = req.message;

	ROS_INFO("HMI input received");
  	ROS_INFO("%d , %d", req.channel, req.message);

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

	ros::Publisher PIDTick_Publisher 	= n.advertise<std_msgs::Bool>("PIDTick_Topic", 1000);
	ros::Publisher Motion_Publisher  	= n.advertise<std_msgs::Bool>("Motion_Topic" , 1000);

	ros::Rate loop_rate(.5);

	std_msgs::Bool msg;
	msg.data = true;

	while (ros::ok())
	{
		if(HMI_Input[0] == 1)
		{
			ROS_INFO("PIDTick: %d", msg.data);
			PIDTick_Publisher.publish(msg);
		}
		
	 	ros::spinOnce();
	 	loop_rate.sleep();
	 }

	return 0;
}

