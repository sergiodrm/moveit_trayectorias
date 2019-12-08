/*
 * PuertaTrayectorias.cpp
 *
 *  Created on: Nov 28, 2019
 *      Author: robindtology
 */

#include <moveit_trayectorias/PuertaTrayectorias.h>

vector<geometry_msgs::Pose> girar_picaporte(geometry_msgs::Pose apoyo, geometry_msgs::Pose eje)
{
	Eigen::MatrixXd m(4,4);
	m << 1, 0, 1, 2,
		 0, 1, 0, 0,
		 0, 0, 1, 0.4,
		 0, 0, 0, 1;

	cout << "La matriz: " << m << std::endl;

	vector<geometry_msgs::Pose> w;
	return w;
}


