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
}

MyRobot::MyRobot(const std::string &planning_group)
{
	this->planning_group = planning_group;
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
}

MyRobot::~MyRobot() {
	// TODO Auto-generated destructor stub
}

geometry_msgs::PoseStamped MyRobot::getCurrentPose()
{
	return this->move_group->getCurrentPose();
}

std::vector<double> MyRobot::getCurrentRPY()
{
	return this->move_group->getCurrentRPY();
}

void MyRobot::ejecutar()
{
	this->move_group->move();
}

void MyRobot::moveto_userpoint(geometry_msgs::Pose *target)
{
	tf2::Quaternion q;

	geometry_msgs::Pose error_pos;
	error_pos.position.x = this->move_group->getCurrentPose().pose.position.x - target->position.x;
	error_pos.position.y = this->move_group->getCurrentPose().pose.position.y - target->position.y;
	error_pos.position.z = this->move_group->getCurrentPose().pose.position.z - target->position.z;
	q.setRPY(this->getCurrentRPY().at(0), this->getCurrentRPY().at(1), this->getCurrentRPY().at(2));
	target->position = this->getCurrentPose().pose.position;



	std::cout << "------------------------" << std::endl << std::endl;
	std::cout << "Posicion [x, y, z] (m):      \t[" << target->position.x << ", ";
	std::cout << target->position.y << ", " << target->position.z << "]" << std::endl;
	std::cout << "Orientacion [R, P, Y] (rad): \t[" << this->getCurrentRPY().at(0) << ", ";
	std::cout << this->getCurrentRPY().at(1) << ", " << this->getCurrentRPY().at(2) << "]" << std::endl;
	std::list<double>::iterator item;
	std::cout << "Error de posicion [x, y, z]: \t[" << error_pos.position.x << ", ";
	std::cout << error_pos.position.y << ", " << error_pos.position.z << "]" << std::endl;
	std::cout << "Error de posicion (euclidea):\t";
	std::cout << sqrt(pow(error_pos.position.x, 2)+pow(error_pos.position.y,2)+pow(error_pos.position.z,2)) << std::endl;
	std::cout << "Tolerancia articular: " << this->move_group->getGoalJointTolerance() << std::endl;
	std::cout << "Tolerancia posicion: " << this->move_group->getGoalPositionTolerance() << std::endl;
	std::cout << "Tolerancia orientacion: " << this->move_group->getGoalOrientationTolerance() << std::endl << std::endl;
	std::cout << "------------------------" << std::endl;

	char op;
	std::cout << "Cambiar pos (s/n): >> ";
	std::cin >> op;
	if (op == 's'){
		std::cout << "\tx: ";
		std::cin >> target->position.x;
		std::cout << "\ty: ";
		std::cin >> target->position.y;
		std::cout << "\tz: ";
		std::cin >> target->position.z;
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
		target->orientation.x = q.getX();
		target->orientation.y = q.getY();
		target->orientation.z = q.getZ();
		target->orientation.w = q.getW();
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
		if (this->plan_JointTrajectory(*target))
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
		waypoints.push_back(*target);
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

bool MyRobot::plan_JointTrajectory(geometry_msgs::Pose target)
{
	moveit::planning_interface::MoveGroupInterface::Plan plan;
	this->move_group->setPoseTarget(target);
	this->visual_tools->deleteAllMarkers();

	const robot_state::JointModelGroup* joint_model_group = this->move_group->getCurrentState()->getJointModelGroup(this->planning_group.c_str());
	this->visual_tools->prompt("Click Next to keep on the plannification...");
	bool success = this->move_group->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
	this->visual_tools->publishAxisLabeled(target, "pose goal");
	this->visual_tools->publishText(text_pose, "Pose Goal", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
	this->visual_tools->publishTrajectoryLine(plan.trajectory_, joint_model_group);
	this->visual_tools->trigger();
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

	this->visual_tools->deleteAllMarkers();
	this->visual_tools->publishAxisLabeled(waypoints.at(waypoints.size()-1), "pose goal");
	this->visual_tools->publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
	this->visual_tools->publishTrajectoryLine(trajectory,
			this->move_group->getCurrentState()->getJointModelGroup(this->planning_group.c_str()));

	this->visual_tools->publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
	for (std::size_t i = 0; i < waypoints.size(); ++i)
		this->visual_tools->publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);

	this->visual_tools->trigger();
	return plan;
}


