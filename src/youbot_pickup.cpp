#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointValue.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <pluginlib/class_loader.h>

#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <stdlib.h>
#include <stdio.h>

#include <boost/units/io.hpp>
#include <boost/units/systems/si/length.hpp> //For m
#include <boost/units/systems/si/plane_angle.hpp> //For rad

//This project's 
#include <youbot_pickup/youbot_arm.h>

//tf::Point in_point;

int main(int argc, char** argv){

  /*if(argc < 4){
	std::cout << "Type a point: x y z" << std::endl;
   } else {*/
        ros::init(argc, argv, "youbot_pickup");
	ros::NodeHandle nh("~"); // Private
	ros::NodeHandle n;

	// TF
   	tf::Transform goal_tf;
   	tf::TransformListener* tfListener = new tf::TransformListener();
   	tf::StampedTransform a0_to_base_link;
	tf::StampedTransform a5_to_a0;

	std::vector<double> initSeed;
	std::vector<double> graspSeed;
	std::vector<double> placeSeed;

	/*in_point.setX(atof(argv[1]));
	in_point.setY(atof(argv[2]));
	in_point.setZ(atof(argv[3]));*/
	
	sleep(2); // For TF
	
	try {
	   tfListener->lookupTransform("arm_link_0", "base_link", ros::Time(0), a0_to_base_link);
	   tfListener->lookupTransform("arm_link_5", "arm_link_0", ros::Time(0), a5_to_a0);

	} catch(tf::LookupException e){
	    ROS_ERROR("Error looking the TF. Error: %s", e.what());
	}

	
	tf::Vector3 t;
	t.setValue(0.341, 0.006, 0.047);
	
	goal_tf.setOrigin(t); //in_point
	
	tf::Quaternion rotation(-0.051, 0.999, -0.000, 0.013);
	
	goal_tf.setRotation(rotation);

	//goal_tf = a5_to_a0 * goal_tf;

	YoubotArm* youbotArm = new YoubotArm(n);

	/*-- Pre-grasp position. --*/
	           
	// Seed values. Obtained with rviz (arm simulator).
	initSeed.push_back(2.95);
	initSeed.push_back(2.32);
	initSeed.push_back(-2.18);
	initSeed.push_back(3.36);
	initSeed.push_back(3.00);

	          

	     /*seed.push_back(0.00010);
	     seed.push_back(0.0);
	     seed.push_back(0.00015);
	     seed.push_back(0.00022);
	     seed.push_back(0.00922);*/

	youbotArm->moveArm(initSeed, goal_tf);
	youbotArm->publishGripperValues(0.0115); // Open the gripper with the move of the arm.				
		
	     

	sleep(5); // Wait for the arm to adopt the pose 

	/*-- Go to the grasp position. --*/
	graspSeed.push_back(2.95);
	graspSeed.push_back(2.37);
	graspSeed.push_back(-1.64);
	graspSeed.push_back(2.77);
	graspSeed.push_back(3.00);
	
     	t.setValue(0.321, 0.005, -0.038);
	goal_tf.setOrigin(t);

	rotation.setValue(-0.051, 0.999, -0.000, 0.009);
	goal_tf.setRotation(rotation);
    	     
	youbotArm->moveArm(graspSeed, goal_tf);	
	sleep(5); // Wait for reseach the goal.    
	
	// Grasping the object.
	youbotArm->publishGripperValues(0.0);		      

	sleep(5); // Wait until the gripper opens.

	/*-- Place the object in the plataform. --*/
	placeSeed.push_back(3.09);
	placeSeed.push_back(-0.0);
	placeSeed.push_back(-2.61);
	placeSeed.push_back(0.017);
	placeSeed.push_back(2.92);

	t.setValue(-0.227, 0.034, 0.100);
	goal_tf.setOrigin(t);

	rotation.setValue(0.067, 0.994, 0.005, -0.083);
	goal_tf.setRotation(rotation);
	
	youbotArm->moveArm(placeSeed, goal_tf);
	sleep(2); // Wait to prevent falls.

	youbotArm->publishGripperValues(0.0115);

	sleep(2); 
	youbotArm->goHome();

	delete youbotArm;
	youbotArm = 0;
   
	return 0;

   //}
}

