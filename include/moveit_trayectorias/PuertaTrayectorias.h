/*
 *	Funciones para generar las trayectorias de
 *	la apertura de puertas
 */

#include <Eigen/Dense>
#include <tf/tf.h>
#include <iostream>
#include <geometry_msgs/Pose.h>

using namespace std;
using namespace Eigen;

void transform2pose(Transform<double, 3, Affine> t, geometry_msgs::Pose &pose);

void pose2RPY(geometry_msgs::Pose pose, double &roll, double &pitch, double &yaw);

void generar_trayectoria_circular(geometry_msgs::Pose apoyo, geometry_msgs::Pose eje, double aplha, vector<geometry_msgs::Pose> &_w);





