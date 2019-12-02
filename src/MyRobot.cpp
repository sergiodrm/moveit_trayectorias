/*
 * MyRobot.cpp
 *
 *  Created on: 14 nov. 2019
 *      Author: sergio
 */

#include <moveit_trayectorias/MyRobot.h>

/* ~~~~~~~~~~~~~~~~~~ Callback para leer el joint_states ~~~~~~~~~~~~~~~~~~~~~~ */
static sensor_msgs::JointState joint_states_robot;

void callback_jointstates(const sensor_msgs::JointState msg)
{
	if (msg.name[0].compare("j2s7s200_joint_1") == 0)
	{
		joint_states_robot = msg;
	}
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */


/* Funcion de valor absoluto */
template<class T>
T my_abs(T n)
{
	if (n < 0)
	{
		return -n;
	}
	return n;
}


MyRobot::MyRobot() {
	// TODO Auto-generated constructor stub
	this->planning_group = "j2s7s200_arm";
	this->move_group = new moveit::planning_interface::MoveGroupInterface(this->planning_group.c_str());
	this->move_group->setPlanningTime(this->plan_time);
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
	this->home.orientation.y = q.getY();
	this->home.orientation.z = q.getZ();
	this->home.orientation.w = q.getW();

	ros::NodeHandle n;
	this->plan_topic = n.advertise<moveit_msgs::RobotTrajectory>("/rb1/my_plan_SJG", 10);
	this->grip = n.advertise<control_msgs::GripperCommandActionGoal>("/rb1/j2s7s200_gripper/gripper_command/goal", 5);
	this->joint_states_subs = n.subscribe("/rb1/joint_state", 20, callback_jointstates);

}

MyRobot::MyRobot(const std::string &planning_group)
{
	this->planning_group = planning_group;
	this->move_group = new moveit::planning_interface::MoveGroupInterface(this->planning_group.c_str());
	this->move_group->setPlanningTime(this->plan_time);
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
	q.normalize();

	std::cout << M_PI_2 << std::endl;

	this->home.position.x = 0.5;
	this->home.position.y = 0;
	this->home.position.z = 0.6;
	this->home.orientation.x = q.getX();
	this->home.orientation.y = q.getY();
	this->home.orientation.z = q.getZ();
	this->home.orientation.w = q.getW();

	this->target = this->home;

	ros::NodeHandle n;
	this->plan_topic = n.advertise<moveit_msgs::RobotTrajectory>("/rb1/my_plan_SJG", 10);
	this->grip = n.advertise<control_msgs::GripperCommandActionGoal>("/rb1/j2s7s200_gripper/gripper_command/goal", 5);
	this->joint_states_subs = n.subscribe("/rb1/joint_states", 20, callback_jointstates);

}

MyRobot::~MyRobot() {
	// TODO Auto-generated destructor stub
}

void MyRobot::print_state()
{
	tf::Quaternion goal_q, state_q, error_q;
	goal_q.setX(this->target.orientation.x);
	goal_q.setY(this->target.orientation.y);
	goal_q.setZ(this->target.orientation.z);
	goal_q.setW(this->target.orientation.w);
	goal_q.normalize();

	state_q.setX(this->move_group->getCurrentPose().pose.orientation.x);
	state_q.setY(this->move_group->getCurrentPose().pose.orientation.y);
	state_q.setZ(this->move_group->getCurrentPose().pose.orientation.z);
	state_q.setW(this->move_group->getCurrentPose().pose.orientation.w);
	state_q.normalize();

	/* CALCULO DEL CUATERNIO DE ERROR ---> q1 · q2* = qe */
	error_q = goal_q;
	error_q.dot(state_q.inverse());

	geometry_msgs::Vector3 error_posicion;
	error_posicion.x = this->target.position.x - this->move_group->getCurrentPose().pose.position.x;
	error_posicion.y = this->target.position.y - this->move_group->getCurrentPose().pose.position.y;
	error_posicion.z = this->target.position.z - this->move_group->getCurrentPose().pose.position.z;


	std::cout << "\033[1;34mPosicion y orientacion actual:\033[0m" << std::endl;
	std::cout << "\tPosicion [x, y, z] (m):      \t[" << this->move_group->getCurrentPose().pose.position.x << ", ";
	std::cout << this->move_group->getCurrentPose().pose.position.y << ", " << this->move_group->getCurrentPose().pose.position.z << "]" << std::endl;
	std::cout << "\tOrientacion [R, P, Y] (rad): \t[" << this->move_group->getCurrentRPY().at(0) << ", ";
	std::cout << this->move_group->getCurrentRPY().at(1) << ", " << this->move_group->getCurrentRPY().at(2) << "]" << std::endl << std::endl;

	std::cout << "\033[1;34mObjetivo actual:\033[0m" << std::endl;
	std::cout << "\tPosicion [x, y, z] (m):      \t[" << this->target.position.x << ", ";
	std::cout << this->target.position.y << ", " << this->target.position.z << "]" << std::endl;
	//	std::cout << "\tOrientacion [R, P, Y] (rad): \t[" << roll << ", " << pitch << ", " << yaw << "]" << std::endl << std::endl;

	std::cout << "\033[1;34mError entre objetivo y posicion actual:\033[0m" << std::endl;
	std::cout << "\tError de posicion [x, y, z]: \t[" << error_posicion.x << ", ";
	std::cout << error_posicion.y << ", " << error_posicion.z << "]" << std::endl;
	std::cout << "\tError de posicion (euclidea):\t";
	std::cout << sqrt(pow(error_posicion.x, 2)+pow(error_posicion.y,2)+pow(error_posicion.z,2)) << " metros" << std::endl;
	std::cout << "\tError de orientacion:        \t" << 2*acos(error_q.getW()) << std::endl;

	//	/* --------------------- Imprimir los joint_states del topico y de la planificación ----------------------------------*/
	//	if (!joint_states_robot.position.empty() && !my_plan.trajectory_.joint_trajectory.points.empty())
	//	{
	//		std::cout << "\033[1;34mPosiciones articulares actuales: (joint_states ··· plan)\033[0m" << std::endl;
	//		for (int i = 0; i < my_plan.trajectory_.joint_trajectory.points[0].positions.size(); i++)
	//		{
	//			if (my_abs<double>(joint_states_robot.position[i] - my_plan.trajectory_.joint_trajectory.points[0].positions[i]) >= 0.01)
	//			{
	//				std::cout << "\033[33;1m";
	//			}
	//			std::cout << joint_states_robot.name[i] << ": " << joint_states_robot.position[i] << " rads\t ···\t ";
	//			std::cout << my_plan.trajectory_.joint_trajectory.joint_names[i] << ": " << my_plan.trajectory_.joint_trajectory.points[0].positions[i] << " rads";
	//			std::cout << "\t" << my_abs<double>(joint_states_robot.position[i] - my_plan.trajectory_.joint_trajectory.points[0].positions[i]) << "\n";
	//			std::cout << "\033[0m";
	//		}
	//		std::cout << std::endl;
	//	} else std::cout << "Not joint_states yet\n";
}

void MyRobot::moveto_userpoint()
{

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

		tf2::Quaternion q;
		q.setRPY(this->move_group->getCurrentRPY().at(0), this->move_group->getCurrentRPY().at(1), this->move_group->getCurrentRPY().at(2));
		q.setRPY(r,p,y);
		q.normalize();
		this->target.orientation.x = q.getX();
		this->target.orientation.y = q.getY();
		this->target.orientation.z = q.getZ();
		this->target.orientation.w = q.getW();
	}


	int tipo;
	std::cout << "Trayectoria articular o cartesiana? (" << tipo_trayectoria::articular << "/" << tipo_trayectoria::cartesiana << ") >>> ";
	std::cin >> tipo;

	if (tipo != tipo_trayectoria::articular && tipo != tipo_trayectoria::cartesiana)
	{
		std::cout << "\033[33mTipo de trayectoria no valida...\033[0m\n";
	} else {
		std::vector<geometry_msgs::Pose> w;
		w.push_back(this->target);
		if (this->plan_Trajectory(w, tipo))
		{
			std::cout << "\033[32mPlanificacion realizada con exito!\033[0m\n";
			this->ejecutar();
		} else std::cout << "\033[31mPlanificacion fallida...\033[0m\n";
	}
}

void MyRobot::corregir_error_final()
{
	geometry_msgs::Vector3 error_ejes;
	std::vector<geometry_msgs::Pose> w;
	char op;

	error_ejes.x = this->target.position.x - this->move_group->getCurrentPose().pose.position.x;
	error_ejes.y = this->target.position.y - this->move_group->getCurrentPose().pose.position.y;
	error_ejes.z = this->target.position.z - this->move_group->getCurrentPose().pose.position.z;


	while (sqrt(pow(error_ejes.x,2) + pow(error_ejes.y,2) + pow(error_ejes.z,2)) >= this->tolerancia_error && op != 'n')
	{
		std::cout << "\033[33mError permitido de " << this->tolerancia_error << " metros." << std::endl;
		std::cout << "\033[33mError de: " << sqrt(pow(error_ejes.x,2) + pow(error_ejes.y,2) + pow(error_ejes.z,2));
		std::cout << " metros...\n\n";
		w.push_back(this->target);
		this->plan_Trajectory(w, tipo_trayectoria::articular);
		w.pop_back();


		std::cout << "Corregir error? (s/n) >>> ";
		std::cin >> op;
		if (op == 's')
		{
			this->move_group->move();
			error_ejes.x = this->target.position.x - this->move_group->getCurrentPose().pose.position.x;
			error_ejes.y = this->target.position.y - this->move_group->getCurrentPose().pose.position.y;
			error_ejes.z = this->target.position.z - this->move_group->getCurrentPose().pose.position.z;
		}
		std::cout << "\033[0m\n";
	}
}

void MyRobot::prueba_precision()
{
	/*
	 * Configuracion de los parametros de la prueba:
	 * 	· delta -> longitud de la trayectoria en metros
	 * 	· n -> numero de trayectorias a realizar
	 * 	· eje -> numero que indica el eje sobre el que se hace la prueba
	 * 	· tipo -> numero que indica si la trayectoria es tipo cartesiana o articular
	 */
	double delta;
	int n, eje, tipo;
	std::cout << "\033[1;34mParametros de la prueba de precision:\n\033[0m";

	std::cout << "\t·Longitud de trayectorias (m): ";
	std::cin >> delta;

	std::cout << "\t·Numero de trayectorias: ";
	std::cin >> n;

	std::cout << "\t·Eje sobre el que realizar las trayectorias:\n";
	std::cout << "\t\t0. X\n\t\t1. Y\n\t\t2. Z\n\t\t3. Diagonal arriba izquierda\n";
	std::cout << "\t\t4. Diagonal arriba derecha\n\t\t5. Diagonal abajo izquierda\n";
	std::cout << "\t\t6. Diagonal abajo derecha\n\n>>> ";
	std::cin >> eje;

	std::cout << "\t·Tipo de trayectoria:\n\t\t0. Articular\n\t\t1. Cartesiana\n\n>>> ";
	std::cin >> tipo;
	std::cout << "Comenzando prueba de precision..." << std::endl;
	this->come_back_home();
	std::vector<geometry_msgs::Pose> w;
	for (int t = 0; t < 3; t++)
	{
		std::cout << "Pruebas para tolerancias:\n" <<
				"\t·Articular:\t" << this->move_group->getGoalJointTolerance() << std::endl <<
				"\t·Posicion:\t" << this->move_group->getGoalPositionTolerance() << std::endl <<
				"\t·Orientacion:\t" << this->move_group->getGoalOrientationTolerance() << std::endl << std::endl;
		for (int i = 0; i<n; i++)
		{
			std::cout << "Numero de trayectoria: #" << i << std::endl;
			/* Ida */
			switch (eje)
			{
			case ejes::eje_x:
				this->target.position.x += delta;
				break;
			case ejes::eje_y:
				this->target.position.y += delta;
				break;
			case ejes::eje_z:
				this->target.position.z += delta;
				break;
			case ejes::diagonal_ari:
				this->target.position.x += delta;
				this->target.position.y += delta;
				this->target.position.z += delta;
				break;
			case ejes::diagonal_ard:
				this->target.position.x += delta;
				this->target.position.y -= delta;
				this->target.position.z += delta;
				break;
			case ejes::diagonal_abi:
				this->target.position.x += delta;
				this->target.position.y += delta;
				this->target.position.z -= delta;
				break;
			case ejes::diagonal_abd:
				this->target.position.x += delta;
				this->target.position.y -= delta;
				this->target.position.z -= delta;
				break;
			}

			/* Planificacion y ejecucion */

			geometry_msgs::Pose actual = this->move_group->getCurrentPose().pose;
			w.push_back(actual);
			w.push_back(this->target);
			this->plan_Trajectory(w, tipo);
			w.clear();

			this->print_state();
			this->ejecutar();

			/* Vuelta */
			switch (eje)
			{
			case ejes::eje_x:
				this->target.position.x -= delta;
				break;
			case ejes::eje_y:
				this->target.position.y -= delta;
				break;
			case ejes::eje_z:
				this->target.position.z -= delta;
				break;
			case ejes::diagonal_ari:
				this->target.position.x -= delta;
				this->target.position.y -= delta;
				this->target.position.z -= delta;
				break;
			case ejes::diagonal_ard:
				this->target.position.x -= delta;
				this->target.position.y += delta;
				this->target.position.z -= delta;
				break;
			case ejes::diagonal_abi:
				this->target.position.x -= delta;
				this->target.position.y -= delta;
				this->target.position.z += delta;
				break;
			case ejes::diagonal_abd:
				this->target.position.x -= delta;
				this->target.position.y += delta;
				this->target.position.z += delta;
				break;
			}

			/* Planificacion y ejecucion */
			actual = this->move_group->getCurrentPose().pose;
			w.push_back(actual);
			w.push_back(this->target);
			w.push_back(this->target);
			this->plan_Trajectory(w, tipo);
			w.clear();

			this->print_state();
			this->ejecutar();
		}
		this->move_group->setGoalJointTolerance(this->move_group->getGoalJointTolerance()*0.1);
		this->move_group->setGoalOrientationTolerance((this->move_group->getGoalOrientationTolerance()*0.1));
		this->move_group->setGoalPositionTolerance(this->move_group->getGoalPositionTolerance()*0.1);
	}

	std::cout << "Fin de la prueba de precision..." << std::endl;
}

void MyRobot::draw_trajectory(std::vector<geometry_msgs::Pose> waypoints)
{
	this->visual_tools->deleteAllMarkers();
	this->visual_tools->publishAxisLabeled(waypoints.at(waypoints.size()-1), "Pose goal");
	this->visual_tools->publishText(text_pose, "Joint trajectory", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
	this->visual_tools->publishTrajectoryLine(my_plan.trajectory_,
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
	std::cout << "\nPlanificando posicion home...\n\n";

	this->target = this->home;
	this->move_group->setPoseTarget(this->target);

	if (this->move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
	{
		ROS_INFO("Plannification successful.");
		std::vector<geometry_msgs::Pose> waypoints;
		waypoints.push_back(this->target);
		this->draw_trajectory(waypoints);

		this->ejecutar();
	}

}

void MyRobot::ejecutar(bool corregir_error)
{
	char op;
	std::cout << "Ejecutar? (s/n) >>> ";
	std::cin >> op;
	if (op == 's')
	{
		this->move_group->execute(my_plan);
		if (corregir_error)
			this->corregir_error_final();
	}
}

bool MyRobot::plan_Trajectory(std::vector<geometry_msgs::Pose> waypoints, int tipo)
{
	bool success = false;
	bool plan_correct;
	int n = 0;
	double tolerance = 0;
	do {
		plan_correct = true;
		if (tipo == tipo_trayectoria::articular)
		{
			this->move_group->setPoseTarget(waypoints.at(waypoints.size()-1));
			success = this->move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
			this->draw_trajectory(waypoints);

		} else if (tipo == tipo_trayectoria::cartesiana) {
			int n = 0;
			do {
				double fraction = this->move_group->computeCartesianPath(waypoints, this->eef_step, this->jump_threshold, my_plan.trajectory_);
				success = fraction != 0 && fraction != -1;
				if (!success)
				{
					std::cout << "\033[31m";
				}
				std::cout << "* Fraction of path trajectory: " << fraction << "\033[0m" << std::endl;
				this->draw_trajectory(waypoints);
				n++;
			} while (!success && n < 10);

			if (!success)
			{
				std::cout << "\033[33m\n\nSe ha probado " << n << " veces la planificacion de la trayectoria"
						"y no se ha podido generar un resultado para el objetivo deseado.\n\n" << "\033[0m";
			}
		}

		/* --------------------- Imprimir los joint_states del topico y de la planificación ----------------------------------*/
		if (!joint_states_robot.position.empty() && !my_plan.trajectory_.joint_trajectory.points.empty())
		{
			ros::param::get("/rb1/move_group/trajectory_execution/allowed_start_tolerance", tolerance);
			std::cout << "\033[1;34mPosiciones articulares actuales: (joint_states ··· plan) (tolerance: " << tolerance << ")\033[0m" << std::endl;
			for (int i = 0; i < my_plan.trajectory_.joint_trajectory.points[0].positions.size(); i++)
			{

				if (my_abs<double>(joint_states_robot.position[i] - my_plan.trajectory_.joint_trajectory.points[0].positions[i]) >=	tolerance)
				{
					std::cout << "\033[33;1m";
					plan_correct = false;
				}
				std::cout << joint_states_robot.name[i] << ": " << joint_states_robot.position[i] << " rad\t···\t";
				std::cout << my_plan.trajectory_.joint_trajectory.joint_names[i] << ": " << my_plan.trajectory_.joint_trajectory.points[0].positions[i] << " rad";
				std::cout << "\t\t" << my_abs<double>(joint_states_robot.position[i] - my_plan.trajectory_.joint_trajectory.points[0].positions[i]) << "\n";
				std::cout << "\033[0m";
			}
			std::cout << std::endl << std::endl;
		} else std::cout << "Not joint_states yet\n";

		if (!plan_correct)
		{
			std::cout << "\033[33mWarning! Se ha detectado diferencia entre el inicio de la planificacion y la posicion articular actual!\n"
					"Repitiendo planificacion...\n";
			n++;
		}

	} while (!plan_correct && n < 10);
	if (plan_correct)
	{
		std::cout << "\033[32m\n\nPlanificacion correcta!\033[0m\n\n";

		std::cout << "Publicando resultado en /rb1/my_plan_SJG...\n";
		this->plan_topic.publish(my_plan.trajectory_);

	} else std::cout << "\033[31m\n\nPlanificacion erronea!\033[0m\n\n";
	return success;
}

void MyRobot::grip_control(double position)
{
	control_msgs::GripperCommandActionGoal msg;
	msg.header.frame_id = "j2s7s200_gripper";
	msg.goal.command.position = position;
	this->grip.publish(msg);
}


