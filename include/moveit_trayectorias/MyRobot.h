/*
 * MyRobot.h
 *
 *  Created on: 14 nov. 2019
 *      Author: sergio
 */

#ifndef SRC_MyRobot_H_
#define SRC_MyRobot_H_

#include <math.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>

class MyRobot {
public:
	MyRobot();
	MyRobot(const std::string &planning_group);
	virtual ~MyRobot();

	void moveto_userpoint(geometry_msgs::Pose *target);
	void draw_trajectory(moveit_msgs::RobotTrajectory trajectory, std::vector<geometry_msgs::Pose> waypoints);
	void print_state();
	void ejecutar();

	void come_back_home();
	bool plan_JointTrajectory(geometry_msgs::Pose target);
	moveit::planning_interface::MoveGroupInterface::Plan plan_CartesianTrajectory(std::vector<geometry_msgs::Pose> target);
private:
	// Nombre de referencia para el brazo
	std::string planning_group;

	// Clase donde se almacenan los metodos para el control del robot
	moveit::planning_interface::MoveGroupInterface *move_group;

	// Clase que permite mostrar mensajes y gráficos en RViz
	moveit_visual_tools::MoveItVisualTools *visual_tools;
	Eigen::Affine3d text_pose;
	geometry_msgs::Pose home;
	geometry_msgs::Pose error;

	const double eef_step = 0.01;
	const double jump_threshold = 0;
};

#endif /* SRC_MyRobot_H_ */
