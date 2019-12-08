/*
 * node_trayectorias.cpp
 *
 *  Created on: 8 dic. 2019
 *      Author: sergio
 */

#include <geometry_msgs/Pose.h>
#include <Eigen/Dense>
#include <ros/ros.h>


std::vector<geometry_msgs::Pose> girar_picaporte(geometry_msgs::Pose apoyo, geometry_msgs::Pose eje);

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Nodo_trayectorias");

	geometry_msgs::Pose apoyo;
	geometry_msgs::Pose eje;
	girar_picaporte(apoyo, apoyo);

	return 0;
}


std::vector<geometry_msgs::Pose> girar_picaporte(geometry_msgs::Pose apoyo, geometry_msgs::Pose eje)
{
	Eigen::MatrixXd m1(4,4), m2(4,4);
	m1 << 1, 0, 1, 2,
		 0, 1, 0, 0,
		 0, 0, 1, 0.4,
		 0, 0, 0, 1;
	m2 << 0, 0, 2, 2,
		 1, 1, 0, 0,
		 1.3, 0, 2, 2,
		 0, 0, 0, 1;

	std::cout << "La matriz: \n" << m1 << std::endl << "Multiplicando...\n";

	m1 *= m2;

	std::cout << "La matriz: \n" << m1 << std::endl << "Multiplicando...\n";

	std::vector<geometry_msgs::Pose> w;
	return w;
}

