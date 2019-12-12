/*
 * PuertaTrayectorias.cpp
 *
 *  Created on: Nov 28, 2019
 *      Author: robindtology
 */

#include <moveit_trayectorias/PuertaTrayectorias.h>

void transform2pose(Transform<double, 3, Affine> t, geometry_msgs::Pose &pose)
{
	/* Variables necesarias para extraer la rotacion en quaternios de la matriz t*/
	tf::Matrix3x3 _rot;
	tf::Quaternion _q;

	for (int fil = 0; fil < 3; fil++)
	{
		for (int col = 0; col < 3; col++)
		{
			_rot[fil][col] = t(fil, col);
		}
	}
	_rot.getRotation(_q);

	/* Se introducen los datos en el mensaje de posicion */
	pose.position.x = t.matrix()(0, 3);
	pose.position.y = t.matrix()(1, 3);
	pose.position.z = t.matrix()(2, 3);
	pose.orientation.x = _q.x();
	pose.orientation.y = _q.y();
	pose.orientation.z = _q.z();
	pose.orientation.w = _q.w();
}

void pose2RPY(geometry_msgs::Pose pose, double &roll, double &pitch, double &yaw)
{
	tf::Quaternion _q;
	tf::Matrix3x3 _m;
	_q.setX(pose.orientation.x);
	_q.setY(pose.orientation.y);
	_q.setZ(pose.orientation.z);
	_q.setW(pose.orientation.w);
	_m.setRotation(_q);
	_m.getRPY(roll, pitch, yaw);
}

void generar_trayectoria_circular(geometry_msgs::Pose current_pose, geometry_msgs::Pose eje, double alpha, vector<geometry_msgs::Pose> &_w)
{
	/*
	 * Al llamar a esta funcion se supone que el brazo
	 * ya ha cogido el picaporte con la posicion y orientacion deseada
	 * 1 .- Se calcula la matriz de transformacion de la base al eje de giro
	 * 2 .- Se realiza la rotacion del eje del picaporte
	 * 3 .- Se calcula la traslacion sobre el picaporte para obtener el punto de apoyo
	 * */
	Transform<double, 3, Affine> T;
	Vector3d X, Y, Z;
	int n = 5;
	geometry_msgs::Pose _pose;

	/* Extraer la orientacion actual del eje de giro para el calculo de las rotaciones */
	double roll, pitch, yaw;
	pose2RPY(eje, roll, pitch, yaw);

	/* Primero se realiza la traslacion del sistema y luego se rota */
	T = Translation3d(Vector3d(eje.position.x, eje.position.y, eje.position.z));
	T *= AngleAxisd(roll, Vector3d::UnitX()) * AngleAxisd(pitch, Vector3d::UnitY()) * AngleAxisd(yaw, Vector3d::UnitZ());

	X << T.matrix()(0,0), T.matrix()(1,0), T.matrix()(2,0);
	Y << T.matrix()(0,1), T.matrix()(1,1), T.matrix()(2,1);
	Z << T.matrix()(0,2), T.matrix()(1,2), T.matrix()(2,2);

	cout << X << endl;
	cout << Y << endl;
	cout << Z << endl;
	/* Hay que coger la rotacion de la posicion actual para añadirla a la transformada del efector final */
	pose2RPY(current_pose, roll, pitch, yaw);

	cout << T.matrix() << endl;
	cout << "Punto inicial de la trayectoria" << ": \tpos [" << current_pose.position.x << ", " << current_pose.position.y << ", " << current_pose.position.z << "]\tori [";
	cout << current_pose.orientation.w << ", " << current_pose.orientation.x << ", " << current_pose.orientation.y << ", " << current_pose.orientation.z << "]\n";

	for (int i = 0; i <= n; i++)
	{
		transform2pose(T * AngleAxisd(i*alpha/n, Z) *
				Translation3d(Vector3d(current_pose.position.x - eje.position.x, current_pose.position.y - eje.position.y, current_pose.position.z - eje.position.z)), _pose);


		cout << "Punto de la trayectoria #" << i << ": \tpos [" << _pose.position.x << ", " << _pose.position.y << ", " << _pose.position.z << "]\tori [";
		cout << _pose.orientation.w << ", " << _pose.orientation.x << ", " << _pose.orientation.y << ", " << _pose.orientation.z << "]\n";

		_w.push_back(_pose);
	}
}

















