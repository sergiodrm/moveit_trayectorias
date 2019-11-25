/*
 * MyRobot.cpp
 *
 *  Created on: 14 nov. 2019
 *      Author: sergio
 */

#include <moveit_trayectorias/MyRobot.h>

MyRobot::MyRobot() {
	// TODO Auto-generated constructor stub
	this->planning_group = "j2s7s200_arm";
	this->move_group = new moveit::planning_interface::MoveGroupInterface(this->planning_group.c_str());
	this->visual_tools = new moveit_visual_tools::MoveItVisualTools(this->move_group->getPlanningFrame().c_str());
	this->visual_tools->deleteAllMarkers();
	this->visual_tools->loadRemoteControl();
	this->text_pose = Eigen::Affine3d::Identity();
	this->text_pose.translation().z() = 1.75;

	// We can print the name of the reference frame for this robot.
	ROS_INFO_NAMED("tutorial", "Reference frame: %s", this->move_group->getPlanningFrame().c_str());

	// We can also print the name of the end-effector link for this group.
	ROS_INFO_NAMED("tutorial", "End effector link: %s", this->move_group->getEndEffectorLink().c_str());

	tf2::Quaternion q;
	q.setRPY(0, M_PI_2, 0);

	this->home.position.x = 0.5;
	this->home.position.y = 0;
	this->home.position.z = 0.6;
	this->home.orientation.x = q.getX();
	this->home.orientation.x = q.getY();
	this->home.orientation.x = q.getZ();
	this->home.orientation.x = q.getW();
}

MyRobot::MyRobot(const std::string &planning_group)
{
	this->planning_group = planning_group;
	this->move_group = new moveit::planning_interface::MoveGroupInterface(this->planning_group.c_str());
	this->visual_tools = new moveit_visual_tools::MoveItVisualTools(this->move_group->getPlanningFrame().c_str());
	this->visual_tools->deleteAllMarkers();
	this->visual_tools->loadRemoteControl();
	this->text_pose = Eigen::Affine3d::Identity();
	this->text_pose.translation().z() = 1.2;

	// We can print the name of the reference frame for this robot.
	ROS_INFO_NAMED("tutorial", "Reference frame: %s", this->move_group->getPlanningFrame().c_str());

	// We can also print the name of the end-effector link for this group.
	ROS_INFO_NAMED("tutorial", "End effector link: %s", this->move_group->getEndEffectorLink().c_str());
	tf2::Quaternion q;
	q.setRPY(0, M_PI_2, 0);

	this->home.position.x = 0.5;
	this->home.position.y = 0;
	this->home.position.z = 0.6;
	this->home.orientation.x = q.getX();
	this->home.orientation.x = q.getY();
	this->home.orientation.x = q.getZ();
	this->home.orientation.x = q.getW();

	this->target = this->home;
}

MyRobot::~MyRobot() {
	// TODO Auto-generated destructor stub
}


void MyRobot::ejecutar()
{
	this->move_group->move();
}

void MyRobot::print_state()
{
	tf::Quaternion goal_q;
	goal_q.setX(this->target.orientation.x);
	goal_q.setY(this->target.orientation.y);
	goal_q.setZ(this->target.orientation.z);
	goal_q.setW(this->target.orientation.w);
	goal_q.normalize();
	tf::Matrix3x3 m(goal_q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	double error_roll, error_pitch, error_yaw;
	error_roll = roll - this->move_group->getCurrentRPY().at(0);
	error_pitch = pitch - this->move_group->getCurrentRPY().at(1);
	error_yaw = yaw - this->move_group->getCurrentRPY().at(2);
	double error_orientation = sqrt(pow(error_roll,2)+pow(error_pitch,2)+pow(error_yaw,2));

	geometry_msgs::Vector3 error_posicion;
	error_posicion.x = this->target.position.x - this->move_group->getCurrentPose().pose.position.x;
	error_posicion.y = this->target.position.y - this->move_group->getCurrentPose().pose.position.y;
	error_posicion.z = this->target.position.z - this->move_group->getCurrentPose().pose.position.z;


	std::cout << "------------------------ Posicion y orientacion actual ------------------------" << std::endl;
	std::cout << "Posicion [x, y, z] (m):      \t[" << this->move_group->getCurrentPose().pose.position.x << ", ";
	std::cout << this->move_group->getCurrentPose().pose.position.y << ", " << this->move_group->getCurrentPose().pose.position.z << "]" << std::endl;
	std::cout << "Orientacion [R, P, Y] (rad): \t[" << this->move_group->getCurrentRPY().at(0) << ", ";
	std::cout << this->move_group->getCurrentRPY().at(1) << ", " << this->move_group->getCurrentRPY().at(2) << "]" << std::endl;

	std::cout << "------------------------ Objetivo actual ------------------------" << std::endl;
	std::cout << "Posicion [x, y, z] (m):      \t[" << this->target.position.x << ", ";
	std::cout << this->target.position.y << ", " << this->target.position.z << "]" << std::endl;
	std::cout << "Orientacion [R, P, Y] (rad): \t[" << roll << ", " << pitch << ", " << yaw << "]" << std::endl;

	std::cout << "------------------------ Tolerancias ------------------------" << std::endl;
	std::cout << "Tolerancia articular: " << this->move_group->getGoalJointTolerance() << std::endl;
	std::cout << "Tolerancia posicion: " << this->move_group->getGoalPositionTolerance() << std::endl;
	std::cout << "Tolerancia orientacion: " << this->move_group->getGoalOrientationTolerance() << std::endl << std::endl;

	std::cout << "------------------------ Error entre objetivo y posicion actual ------------------------" << std::endl;
	std::cout << "Error de posicion [x, y, z]: \t[" << error_posicion.x << ", ";
	std::cout << error_posicion.y << ", " << error_posicion.z << "]" << std::endl;
	std::cout << "Error de posicion (euclidea):\t";
	std::cout << sqrt(pow(error_posicion.x, 2)+pow(error_posicion.y,2)+pow(error_posicion.z,2)) << "metros" << std::endl;
	std::cout << "Error de orientacion:        \t" << error_orientation << std::endl;
}

void MyRobot::moveto_userpoint()
{
	tf2::Quaternion q;
	q.setRPY(this->move_group->getCurrentRPY().at(0), this->move_group->getCurrentRPY().at(1), this->move_group->getCurrentRPY().at(2));
	this->print_state();

	char op;
	std::cout << "Cambiar pos (s/n): >> ";
	std::cin >> op;
	if (op == 's'){
		std::cout << "\tx: ";
		std::cin >> this->target.position.x;
		std::cout << "\ty: ";
		std::cin >> this->target.position.y;
		std::cout << "\tz: ";
		std::cin >> this->target.position.z;
	}
	std::cout << "Cambiar orientacion (s/n): >> ";
	std::cin >> op;
	if (op == 's'){
		double r, p ,y;
		std::cout << "\tRoll: ";
		std::cin >> r;
		std::cout << "\tPitch: ";
		std::cin >> p;
		std::cout << "\tYaw: ";
		std::cin >> y;
		q.setRPY(r,p,y);
		q.normalize();
		this->target.orientation.x = q.getX();
		this->target.orientation.y = q.getY();
		this->target.orientation.z = q.getZ();
		this->target.orientation.w = q.getW();
	}
	std::cout << "Cambiar tolerancias? (s/n): >> ";
	std::cin >> op;
	if (op == 's')
	{
		double tolerance;
		std::cout << "Tolerancia articular: >>> ";
		std::cin >> tolerance;
		this->move_group->setGoalJointTolerance(tolerance);

		std::cout << "Tolerancia posicion: >>> ";
		std::cin >> tolerance;
		this->move_group->setGoalPositionTolerance(tolerance);

		std::cout << "Tolerancia orientacion: >>> ";
		std::cin >> tolerance;
		this->move_group->setGoalOrientationTolerance(tolerance);
	}

	std::cout << "Trayectoria articular o cartesiana? (a/c) >>> ";
	std::cin >> op;

	if (op == 'a')
	{
		if (this->plan_JointTrajectory(target))
		{
			ROS_INFO("Plannification successful.");
			std::cout << "Ejecutar (s/n): >> ";

			std::cin >> op;
			if (op == 's')
			{
				this->move_group->move();
			}
		} else {
			ROS_INFO("Plannification failed.");
		}
	} else if (op == 'c') {
		std::vector<geometry_msgs::Pose> waypoints;
		waypoints.push_back(target);
		moveit::planning_interface::MoveGroupInterface::Plan plan;
		plan = this->plan_CartesianTrajectory(waypoints);

		ROS_INFO("Plannification successful.");
		std::cout << "Ejecutar (s/n): >> ";

		std::cin >> op;
		if (op == 's')
		{
			this->move_group->execute(plan);
		}
	}
}

void MyRobot::prueba_precision()
{
	double tramos = 0.2;

	std::cout << "Comenzando prueba de precision..." << std::endl;
	this->come_back_home();
	std::cout << "Planificando movimiento en eje X..." << std::endl;
	geometry_msgs::Pose point = this->move_group->getCurrentPose().pose;
	std::vector<geometry_msgs::Pose> w;

	point.position.x += tramos;
	w.push_back(point);
	this->plan_CartesianTrajectory(w);
	w.pop_back();

	point = this->move_group->getCurrentPose().pose;
	point.position.y += tramos;
	w.push_back(point);
	this->plan_CartesianTrajectory(w);
	w.pop_back();

	point = this->move_group->getCurrentPose().pose;
	point.position.z += tramos;
	w.push_back(point);
	this->plan_CartesianTrajectory(w);
	w.pop_back();

	w.push_back(this->home);
	this->plan_CartesianTrajectory(w);

	point = this->move_group->getCurrentPose().pose;
	point.position.x += tramos;
	point.position.y += tramos;
	point.position.z += tramos;
	this->plan_JointTrajectory(point);

	point.position.y = -point.position.y;
	this->plan_JointTrajectory(point);

	std::cout << "Fin de la prueba de precision..." << std::endl;
}

void MyRobot::draw_trajectory(moveit_msgs::RobotTrajectory trajectory, std::vector<geometry_msgs::Pose> waypoints)
{
	this->visual_tools->deleteAllMarkers();
	this->visual_tools->publishAxisLabeled(waypoints.at(waypoints.size()-1), "Pose goal");
	this->visual_tools->publishText(text_pose, "Joint trajectory", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
	this->visual_tools->publishTrajectoryLine(trajectory,
			this->move_group->getCurrentState()->getJointModelGroup(this->planning_group.c_str()));
	if (!waypoints.empty())
	{
		this->visual_tools->publishPath(waypoints, rviz_visual_tools::LIME_GREEN, rviz_visual_tools::SMALL);
		for (std::size_t i = 0; i < waypoints.size(); ++i)
				this->visual_tools->publishAxisLabeled(waypoints[i], "point " + std::to_string(i), rviz_visual_tools::SMALL);
	}
	this->visual_tools->trigger();
}

void MyRobot::come_back_home()
{
	moveit::planning_interface::MoveGroupInterface::Plan plan;

	ROS_INFO("Planificando posicion home...");

	this->move_group->setPoseTarget(this->home);

	if (this->move_group->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
	{
		ROS_INFO("Plannification successful.");
		std::vector<geometry_msgs::Pose> waypoints;
		waypoints.push_back(this->home);
		this->draw_trajectory(plan.trajectory_, waypoints);
		char op;
		std::cout << "Ejecutar home? (s/n): >>> ";
		std::cin >> op;
		if (op == 's')
			this->move_group->move();
	}

}

bool MyRobot::plan_JointTrajectory(geometry_msgs::Pose target)
{
	moveit::planning_interface::MoveGroupInterface::Plan plan;
	std::vector<geometry_msgs::Pose> waypoints;
	waypoints.push_back(target);
	this->move_group->setPoseTarget(target);
	bool success = this->move_group->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
	this->draw_trajectory(plan.trajectory_, waypoints);
	return success;
}

moveit::planning_interface::MoveGroupInterface::Plan MyRobot::plan_CartesianTrajectory(std::vector<geometry_msgs::Pose> target)
{
	std::vector<geometry_msgs::Pose> waypoints;
	waypoints.push_back(this->move_group->getCurrentPose().pose);

	std::vector<geometry_msgs::Pose>::iterator item;
	for (item = target.begin(); item != target.end(); item ++)
		waypoints.push_back(*item);

	moveit_msgs::RobotTrajectory trajectory;
	moveit::planning_interface::MoveGroupInterface::Plan plan;

	namespace rvt = rviz_visual_tools;
	double fraction = this->move_group->computeCartesianPath(waypoints, this->eef_step,
			this->jump_threshold, trajectory);

	plan.trajectory_ = trajectory;

	this->draw_trajectory(trajectory, waypoints);
	return plan;
}


