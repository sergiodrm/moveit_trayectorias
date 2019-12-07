/*
 * move_group_pruebas.cpp
 *
 *  Created on: 13 nov. 2019
 *      Author: sergio
 */

#include <moveit_trayectorias/MyRobot.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pruebas_precision");
	ros::AsyncSpinner spinner(1);
	spinner.start();


	MyRobot rb1("j2s7s200_arm");
	char op;
	do{
		rb1.prueba_precision();
		std::cout << "Terminar ejecucion? (s/n)\n>>> ";
		std::cin >> op;
	} while (ros::ok() && op != 's');
	ros::shutdown();
	return 0;
}
