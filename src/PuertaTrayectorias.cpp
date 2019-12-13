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
	_q.normalize();

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
	_q.normalize();
	_m.setRotation(_q);
	_m.getRPY(roll, pitch, yaw);
}

void abrir_puerta(geometry_msgs::Pose apoyo, geometry_msgs::Pose eje_picaporte, geometry_msgs::Pose eje_puerta, vector<geometry_msgs::Pose> &waypoints)
{
	Transform<double, 3, Affine> T1, T2, T3, T4, T5;
	geometry_msgs::Pose pose;
	int n = 5;
	double alpha = -M_PI/6;
	double beta = -M_PI/6;


	/* Giro del picaporte */
	T1 = Translation3d(Vector3d(eje_picaporte.position.x, eje_picaporte.position.y, eje_picaporte.position.z));
//	T2 = AngleAxisd(M_PI_2, Vector3d::UnitY());
	T3 = Translation3d(Vector3d(apoyo.position.x - eje_picaporte.position.x,
			apoyo.position.y - eje_picaporte.position.y,
			apoyo.position.z - eje_picaporte.position.z));

	cout << "Giro del picaporte:\n";
	for (int i = 0; i <= n; i++)
	{
		T2 = AngleAxisd(M_PI_2, Vector3d::UnitY()) * AngleAxisd(i*alpha/n, Vector3d::UnitZ());

		transform2pose(T1 * T2 * T3, pose);

		cout << "Punto de la trayectoria #" << i << ": \tpos [" << pose.position.x << ", " << pose.position.y << ", " << pose.position.z << "]\tori [";
		cout << pose.orientation.w << ", " << pose.orientation.x << ", " << pose.orientation.y << ", " << pose.orientation.z << "]\n";

		waypoints.push_back(pose);
	}

	/* Apertura de la puerta */
	T1 = Translation3d(Vector3d(eje_puerta.position.x, eje_puerta.position.y, eje_puerta.position.z));
	T3 = Translation3d(Vector3d(eje_picaporte.position.x - eje_puerta.position.x,
				eje_picaporte.position.y - eje_puerta.position.y,
				eje_picaporte.position.z - eje_puerta.position.z));
	T4 = AngleAxisd(M_PI_2, Vector3d::UnitY()) * AngleAxisd(alpha, Vector3d::UnitZ());
	T5 = Translation3d(Vector3d(apoyo.position.x - eje_picaporte.position.x,
			apoyo.position.y - eje_picaporte.position.y,
			apoyo.position.z - eje_picaporte.position.z));

	cout << "Apertura de la puerta:\n";
	for (int i = 0; i <= n; i++)
	{
		T2 = AngleAxisd(i*beta/n, Vector3d::UnitZ());

		transform2pose(T1 * T2 * T3 * T4 * T5, pose);

		cout << "Punto de la trayectoria #" << i << ": \tpos [" << pose.position.x << ", " << pose.position.y << ", " << pose.position.z << "]\tori [";
		cout << pose.orientation.w << ", " << pose.orientation.x << ", " << pose.orientation.y << ", " << pose.orientation.z << "]\n";

		waypoints.push_back(pose);
	}

	/* Deshacer giro del picaporte */
//	alpha = -alpha;
	T2 = AngleAxisd(beta, Vector3d::UnitZ());

	cout << "Deshacer giro del picaporte:\n";
	for (int i = n; i >= 0; i--)
	{
		T4 = AngleAxisd(M_PI_2, Vector3d::UnitY()) * AngleAxisd(i*alpha/n, Vector3d::UnitZ());

		transform2pose(T1 * T2 * T3 * T4 * T5, pose);

		cout << "Punto de la trayectoria #" << i << ": \tpos [" << pose.position.x << ", " << pose.position.y << ", " << pose.position.z << "]\tori [";
		cout << pose.orientation.w << ", " << pose.orientation.x << ", " << pose.orientation.y << ", " << pose.orientation.z << "]\n";

		waypoints.push_back(pose);
	}

}

void girar_picaporte(geometry_msgs::Pose current_pose, geometry_msgs::Pose eje, double alpha, vector<geometry_msgs::Pose> &_w)
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

	/*
	 * Primero se realiza la traslacion del sistema y luego se rota
	 * 		Traslacion + Rotacion sobre movil ---> pos multiplicacion
	 */
	T = Translation3d(Vector3d(eje.position.x, eje.position.y, eje.position.z));
	T *= AngleAxisd(roll, Vector3d::UnitX()) * AngleAxisd(pitch, Vector3d::UnitY()) * AngleAxisd(yaw, Vector3d::UnitZ());

	/* Hay que coger la rotacion de la posicion actual para añadirla a la transformada del efector final */
	pose2RPY(current_pose, roll, pitch, yaw);

	cout << T.matrix() << endl;
	cout << "Punto inicial de la trayectoria" << ": \tpos [" << current_pose.position.x << ", " << current_pose.position.y << ", " << current_pose.position.z << "]\tori [";
	cout << current_pose.orientation.w << ", " << current_pose.orientation.x << ", " << current_pose.orientation.y << ", " << current_pose.orientation.z << "]\n";

	for (int i = 0; i <= n; i++)
	{
		transform2pose(T * AngleAxisd(i*alpha/n, Vector3d::UnitZ()) *
				Translation3d(Vector3d(current_pose.position.x - eje.position.x, current_pose.position.y - eje.position.y, current_pose.position.z - eje.position.z)), _pose);

		cout << "Punto de la trayectoria #" << i << ": \tpos [" << _pose.position.x << ", " << _pose.position.y << ", " << _pose.position.z << "]\tori [";
		cout << _pose.orientation.w << ", " << _pose.orientation.x << ", " << _pose.orientation.y << ", " << _pose.orientation.z << "]\n";

		_w.push_back(_pose);
	}
}

void girar_puerta(geometry_msgs::Pose current_pose, geometry_msgs::Pose eje, double alpha, vector<geometry_msgs::Pose> &_w)
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

	/*
	 * Primero se realiza la traslacion del sistema y luego se rota
	 * 		Traslacion + Rotacion sobre movil ---> pos multiplicacion
	 */
	T = Translation3d(Vector3d(eje.position.x, eje.position.y, eje.position.z));
	T *= AngleAxisd(roll, Vector3d::UnitX()) * AngleAxisd(pitch, Vector3d::UnitY()) * AngleAxisd(yaw, Vector3d::UnitZ());

	/* Hay que coger la rotacion de la posicion actual para añadirla a la transformada del efector final */
	pose2RPY(current_pose, roll, pitch, yaw);
	cout << roll << " " << pitch << " " << yaw << endl;

	cout << "Punto inicial de la trayectoria" << ": \tpos [" << current_pose.position.x << ", " << current_pose.position.y << ", " << current_pose.position.z << "]\tori [";
	cout << current_pose.orientation.w << ", " << current_pose.orientation.x << ", " << current_pose.orientation.y << ", " << current_pose.orientation.z << "]\n";

	for (int i = 0; i <= n; i++)
	{
		transform2pose(T * AngleAxisd(i*alpha/n, Vector3d::UnitZ()) *
				Translation3d(Vector3d(current_pose.position.x - eje.position.x, current_pose.position.y - eje.position.y, current_pose.position.z - eje.position.z)) *
						AngleAxisd(roll, Vector3d::UnitX()) * AngleAxisd(pitch, Vector3d::UnitY()) * AngleAxisd(yaw, Vector3d::UnitZ()), _pose);

		cout << "Punto de la trayectoria #" << i << ": \tpos [" << _pose.position.x << ", " << _pose.position.y << ", " << _pose.position.z << "]\tori [";
		cout << _pose.orientation.w << ", " << _pose.orientation.x << ", " << _pose.orientation.y << ", " << _pose.orientation.z << "]\n";

		_w.push_back(_pose);
	}
}















