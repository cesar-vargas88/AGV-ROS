#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "Minotauro/HMI.h"
#include <iostream>

int main(int argc, char **argv)
{
  	ros::init(argc, argv, "HMI_Input_Node");
  	ros::NodeHandle n;

	ros::ServiceClient HMI_Input_Client = n.serviceClient<Minotauro::HMI>("HMI_Input_Service");

	Minotauro::HMI srv;

	while(true)
	{
		std::cout << "Input a command: ";
		std::cin >> srv.request.message;
		srv.request.channel = 0;

		HMI_Input_Client.call(srv);
			
		ros::spinOnce();
	}

  	return 0;
}
