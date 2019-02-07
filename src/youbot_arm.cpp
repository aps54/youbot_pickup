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

//using namespace youbot_arm_kinematics_moveit;

tf::Point in_point;
tf::Transform goal_tf;


//geometry_msgs::Pose aprx_pose;

brics_actuator::JointPositions pos;

std::vector<double> seed;
std::vector<brics_actuator::JointValue> armJointPositions;
std::vector<brics_actuator::JointValue> gripperJointPositions;

// Publishers 
ros::Publisher arm_pub, gripper_pub;

void publishArmValues(std::vector<double>& p){
   
   std::stringstream jointName;
   armJointPositions.resize(p.size());

   for (unsigned int i=0; i < p.size(); i++){
	jointName.str("");
	jointName << "arm_joint_" << (i+1);
	armJointPositions[i].joint_uri = jointName.str();
	armJointPositions[i].unit = boost::units::to_string(boost::units::si::radians);
	armJointPositions[i].value = p[i];
   }

   pos.positions = armJointPositions;
   arm_pub.publish(pos);

   ROS_INFO("END publishArmValues");
}

void publishGripperValues(double w){

   gripperJointPositions.resize(2);

   gripperJointPositions[0].joint_uri = "gripper_finger_joint_l";
   gripperJointPositions[1].joint_uri = "gripper_finger_joint_r";

   gripperJointPositions[0].unit = boost::units::to_string(boost::units::si::meter);
   gripperJointPositions[1].unit = boost::units::to_string(boost::units::si::meter);

   gripperJointPositions[0].value = w;
   gripperJointPositions[0].value = w;

   pos.positions = gripperJointPositions;
   gripper_pub.publish(pos);
}

int main(int argc, char** argv){

   geometry_msgs::Pose goal_pose;
   tf::Quaternion q;

   if(argc < 4){
	std::cout << "Type a point: x y z" << std::endl;
   } else {
        ros::init(argc, argv, "youbot_pickup");
	ros::NodeHandle nh("~"); // Private
	ros::NodeHandle n;

	in_point.setX(atof(argv[1]));
	in_point.setY(atof(argv[2]));
	in_point.setZ(atof(argv[3]));

	tf::Matrix3x3 base;
	base.setValue(0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0);
	tf::Vector3 t;
	t.setValue(0.310676, 0.00250788, 0.351227);
	goal_tf.setOrigin(t); //in_point
	goal_tf.setBasis(base);
	goal_tf.setRotation(tf::Quaternion::getIdentity());

	//tf::Vector3& p = goal_tf.getOrigin();
	//goal_pose.delete();
	goal_pose.position.x = t.getX();//in_point.getX();
	goal_pose.position.y = t.getY();//in_point.getY();
	goal_pose.position.z = t.getZ();//in_point.getZ();

	q = goal_tf.getRotation();

	goal_pose.orientation.w = q.getW();
	goal_pose.orientation.x = q.getX();
	goal_pose.orientation.y = q.getY();
	goal_pose.orientation.z = q.getZ();

	ROS_INFO("Goal pose: ");
	std::cout << "Position(x, y, z) = (" <<  goal_pose.position.x << ", " << goal_pose.position.y << ", " << goal_pose.position.z << ")" << std::endl;
	std::cout << "Rotation(w, x, y, z) = (" << goal_pose.orientation.w << ", " << goal_pose.orientation.x << ", " << goal_pose.orientation.y << ", " << goal_pose.orientation.z << ")" << std::endl;

	// Publishers
	arm_pub = n.advertise<brics_actuator::JointPositions>("arm_1/arm_controller/position_command", 1);
	gripper_pub = n.advertise<brics_actuator::JointPositions>("arm_1/gripper_controller/position_command", 1);

        // Instance & initialize IK solver
	pluginlib::ClassLoader<kinematics::KinematicsBase> loader("moveit_core", "kinematics::KinematicsBase");
	try
	{	
	     boost::shared_ptr<kinematics::KinematicsBase> ik_ = loader.createInstance("youbot_arm_kinematics_moveit::KinematicsPlugin");
	     ik_->initialize("/robot_description", "arm_1", "arm_link_0", "arm_link_5", 0.1);
	   
	     std::vector<double> solution;
	     moveit_msgs::MoveItErrorCodes error_code;
	   
	     // Seed values. Obtained with rviz (moveit plugin).
	     /*seed.push_back(2.83552);
	     seed.push_back(2.20516);
	     seed.push_back(-1.77035);
	     seed.push_back(3.00807);
	     seed.push_back(0.10603);*/

	     seed.push_back(0.00010);
	     seed.push_back(0.0);
	     seed.push_back(0.00015);
	     seed.push_back(0.00022);
	     seed.push_back(0.00922);

	     ik_->getPositionIK(goal_pose, seed, solution, error_code);
	     if(error_code.val == error_code.SUCCESS){
	     ROS_INFO("The solutions are: ");
	     printf("A1. %e; A2. %e; A3. %e; A4. %e; A5. %e;\n", solution[0], solution[1], solution[2], solution[3], solution[4]);
	     //std::cout << solution[0] << ", " << solution[1] << ", " << solution[2] << ", " << solution[3] << ", " << solution[4] << "." << std::endl;
	     /*std::ostringstream strs;
	     strs << solution[0];
	     std::string s = strs.str();
	     ROS_INFO("%s\n", s.c_str());*/
	     } else std::cout << "Error:" << error_code << std::endl;

	     publishArmValues(solution);
	     publishGripperValues(0.0115);
	     sleep(2);
	     publishGripperValues(0.0);

	 }

	 catch(pluginlib::PluginlibException& ex){
	     ROS_ERROR("Error loading the IK plugin. Error: %s", ex.what());
	     return -1;
	 }

   	 return 0;
   }
}

