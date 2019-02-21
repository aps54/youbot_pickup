#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Transform.h>

#include <geometry_msgs/Pose.h>
#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointValue.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <pluginlib/class_loader.h>
//#include <youbot_arm_kinematics_moveit/youbot_arm_kinematics_moveit.h>

#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <stdlib.h>
#include <stdio.h>

#include <boost/units/io.hpp>
#include <boost/units/systems/si/length.hpp> //For m
#include <boost/units/systems/si/plane_angle.hpp> //For rad

class YoubotArm {

	/* Private variables */
	
	brics_actuator::JointPositions pos;

	std::vector<brics_actuator::JointValue> armJointPositions;
	std::vector<brics_actuator::JointValue> gripperJointPositions;

	// Seed values
	std::vector<double> initSeed;
	std::vector<double> graspSeed;
	std::vector<double> placeSeed;

	// Home values
	std::vector<double> homeValues;

	// Transforms
	tf::Transform init_tf;
	tf::Transform grasp_tf;
	tf::Transform place_tf;

	// Publishers 
	ros::Publisher arm_pub, gripper_pub;

  	/* Private functions */

	void publishArmValues(std::vector<double>& p);

	void publishGripperValues(double w);

	int moveArm(std::vector<double>& seed, tf::Transform& goal);

   protected:

	typedef boost::shared_ptr<kinematics::KinematicsBase> KinematicsPtr;
	KinematicsPtr ik_;

	

    public:

	/* Constructor */
	YoubotArm(ros::NodeHandle& n);

	/* Auxiliar functions */
	
	void goHome();
	
	// Gripper functions
	void openGripper();
	void closeGripper();

	// Arm positions functions
	void goToPregrasp();
	void goToGrasp();
	void goToPlace();
	
};
