/*
 * node_trayectorias.cpp
 *
 *  Created on: 8 dic. 2019
 *      Author: sergio
 */

#include <moveit_trayectorias/PuertaTrayectorias.h>
#include <moveit_trayectorias/MyRobot.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Dense>
#include <ros/ros.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "trayectorias");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	std::vector<geometry_msgs::Pose> _w;
	std::vector<double> _e;
	geometry_msgs::Pose _pose, eje_picaporte, eje_puerta, apoyo;
	tf::Quaternion _q;
	_q.setRPY(0, M_PI_2, 0);

	/* Inicializacion de los sistemas de coordenadas de la puerta */
	eje_picaporte.position.x = 0.7;
	eje_picaporte.position.y = 0;
	eje_picaporte.position.z = 1.01;
	eje_picaporte.orientation.x = _q.x();
	eje_picaporte.orientation.y = _q.y();
	eje_picaporte.orientation.z = _q.z();
	eje_picaporte.orientation.w = _q.w();

	apoyo.position.x = 0.7;
	apoyo.position.y = 0.115;
	apoyo.position.z = 1.01;
	apoyo.orientation.x = _q.x();
	apoyo.orientation.y = _q.y();
	apoyo.orientation.z = _q.z();
	apoyo.orientation.w = _q.w();

	_q.setRPY(0, 0, 0);
	_q.normalize();
	eje_puerta.position.x = 0.7;
	eje_puerta.position.y = 0.775;
	eje_puerta.position.z = 1.01;
	eje_puerta.orientation.x = _q.x();
	eje_puerta.orientation.y = _q.y();
	eje_puerta.orientation.z = _q.z();
	eje_puerta.orientation.w = _q.w();

	/*
	 * Se crea el objeto del robot donde se mandarán las trayectorias
	 */
	MyRobot rb1("j2s7s200_arm");

	/* Elegir si se quiere ejecutar los movimientos de manera automatica */
	rb1.setModoAutomatico(false);

	/* Abrir pinza por si se ha quedado abierta de anteriores movimientos */


	/*
	 * Primero se coloca el brazo en la posicion home
	 */

	rb1.come_back_home();

	/*
	 * Colocacion del brazo a la altura del picaporte con la orientacion deseada
	 */
	rb1.grip_control(pinza::abierta);
	_pose = apoyo;
	_pose.position.x -= 0.1;
	_w.push_back(_pose);

	rb1.plan_Trajectory(_w, tipo_trayectoria::articular);
	rb1.ejecutar(true);

	_e.push_back(_pose.position.x);
	_e.push_back(_pose.position.y);
	_e.push_back(_pose.position.z);
	rb1.print_error(_e);

	_w.clear();

	/* Acercamiento al picaporte en linea recta y cerrar la pinza */
	_w.push_back(apoyo);
	rb1.plan_Trajectory(_w, tipo_trayectoria::cartesiana);
	rb1.ejecutar(true);
	rb1.grip_control(pinza::cerrada);

	_e.at(0) = apoyo.position.x;
	_e.at(1) = apoyo.position.y;
	_e.at(2) = apoyo.position.z;
	rb1.print_error(_e);

	_w.clear();

	abrir_puerta(apoyo, eje_picaporte, eje_puerta, _w);
	rb1.plan_Trajectory(_w, tipo_trayectoria::cartesiana);
	rb1.ejecutar(true);
	rb1.grip_control(pinza::abierta);


	ros::shutdown();
	return 0;
}


