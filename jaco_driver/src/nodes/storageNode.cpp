#include <ros/ros.h>
#include <jaco_driver/goStorage.h>

int main(int argc, char **argv)
{
	ros::init(argc,argv,"storageNode");
	goStorage gs;
	ros::spin();

	return(0);
}	
