// Categorical Shape Registration
// Author: Florian Huber <s6flhube@uni-bonn.de>
//         Diego Rodriguez <rodriguez@ais.uni-bonn.de>

#include <shape_registration/shape_registration.hpp>
#include <ros/ros.h>


using namespace categorical_registration;

int main (int argc, char *argv[])
{
	ros::init(argc, argv, "shape_registration_node");
	ros::NodeHandle n;

	ros::Rate loop_rate(30);

	ShapeRegistration surf_reg;

	// ROS loop
	while (n.ok())
	{
		ros::spinOnce();
		surf_reg.viewer()->spinOnce();
		loop_rate.sleep();
	}
}
