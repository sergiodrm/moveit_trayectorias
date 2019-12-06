/*
 * MyRobot.h
 *
 *  Created on: 14 nov. 2019
 *      Author: sergio
 */

#ifndef SRC_MyRobot_H_
#define SRC_MyRobot_H_

#include <math.h>
#include <queue>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <control_msgs/GripperCommandActionGoal.h>
#include <ros/ros.h>

enum ejes {eje_x, eje_y, eje_z, diagonal_ari, diagonal_ard, diagonal_abi, diagonal_abd};
enum tipo_trayectoria {articular, cartesiana};

class MyRobot {
public:
	MyRobot();
	MyRobot(const std::string &planning_group);
	virtual ~MyRobot();

	void moveto_userpoint();
	void draw_trajectory(std::vector<geometry_msgs::Pose> waypoints);
	void print_state();
	void ejecutar(bool corregir_error = false);
	void corregir_error_final();
	void open_door();

	void come_back_home();
	bool plan_Trajectory(std::vector<geometry_msgs::Pose> waypoints, int tipo);
	void prueba_precision();

	void grip_control(double position);
private:
	// Nombre de referencia para el brazo
	std::string planning_group;

	// Clase donde se almacenan los metodos para el control del robot
	moveit::planning_interface::MoveGroupInterface *move_group;
	ros::Rate *rate;

	// Clase que permite mostrar mensajes y gráficos en RViz
	moveit_visual_tools::MoveItVisualTools *visual_tools;
	Eigen::Affine3d text_pose;

	// Variable para leer el joint_states
	ros::Subscriber joint_states_subs;
	sensor_msgs::JointState joint_states;

	// Variables para las trayectorias
	geometry_msgs::Pose home;
	geometry_msgs::Pose target;
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	// Variables para controlar la pinza del brazo
	ros::Publisher grip;

	// Constantes para el planificador
	ros::Publisher plan_topic;
	const double eef_step = 0.01;
	const double jump_threshold = 0;
	const double tolerancia_error = 0.02;
	const double plan_time = 10;
};

#endif /* SRC_MyRobot_H_ */
