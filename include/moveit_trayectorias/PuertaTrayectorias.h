/*
 *	Funciones para generar las trayectorias de
 *	la apertura de puertas
 */

#include <Eigen/Dense>
#include <iostream>
#include <geometry_msgs/Point.h>

using namespace std;

vector<geometry_msgs::Point> aproximacion_picaporte();
