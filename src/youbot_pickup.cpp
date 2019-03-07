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
#include <youbot_pickup/youbot_base.h>

//tf::Point in_point;

int main(int argc, char** argv){
  
    ros::init(argc, argv, "youbot_pickup");
    ros::NodeHandle nh("~"); // Private
    ros::NodeHandle n;

    // TF
    //tf::Transform goal_tf;
    tf::TransformListener* tfListener = new tf::TransformListener();
    tf::StampedTransform a0_to_base_link;
    tf::StampedTransform a0_to_a5;
    tf::StampedTransform camera_to_base;
	
    sleep(2); // For fill TF listener.
	
    try {
	tfListener->lookupTransform("arm_link_0", "base_link", ros::Time(0), a0_to_base_link);
	tfListener->lookupTransform("arm_link_0", "arm_link_5", ros::Time(0), a0_to_a5);
    } catch(tf::LookupException e){
	ROS_ERROR("Error looking the TF. Error: %s", e.what());
    }

	
    YoubotArm* youbotArm = new YoubotArm(n);

    YoubotBase* youbotBase = new YoubotBase(n);

    geometry_msgs::Pose base_goal;
    base_goal.position.x = 0.06373;
    base_goal.position.y = 0.84160;
    base_goal.position.z = 0.87075;

    base_goal.orientation.x = -0.93039;
    base_goal.orientation.y = -0.03312;
    base_goal.orientation.z = 0.36485;
    base_goal.orientation.w = 0.01238;

    youbotBase->publishGoal(base_goal);

    /*-- Pre-grasp position. --        

    youbotArm->goToPregrasp();
    youbotArm->openGripper(); // Open the gripper with the move of the arm.				
		
    sleep(5); // Wait for the arm to adopt the pose 

    /*-- Go to the grasp position. --
    	     
    youbotArm->goToGrasp();	
    sleep(5); // Wait for reseach the goal.    
	
    // Grasping the object.
    youbotArm->closeGripper();		      

    sleep(5); // Wait until the gripper closes.

    /*-- Place the object in the plataform. --
		
    youbotArm->goToPlace();
    sleep(2); // Wait to prevent falls.

    youbotArm->openGripper();
    sleep(5); // Wait until the gripper is all open.

    youbotArm->goHome();
*/
    delete youbotArm;
    youbotArm = 0;
   
    delete youbotBase;
    youbotBase = 0;	
	
    return 0;

   
}

