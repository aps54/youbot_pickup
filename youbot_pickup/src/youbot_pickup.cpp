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
#include <youbot_pickup/qr_detection.h>

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

    QRDetector* qrDetector = new QRDetector();

    //while(ros::ok()){

    geometry_msgs::Pose base_goal;
/*    base_goal.position.x = 0.06373;
    base_goal.position.y = 0.84160;
    base_goal.position.z = 0.87075;

    base_goal.orientation.x = -0.93039;
    base_goal.orientation.y = -0.03312;
    base_goal.orientation.z = 0.36485;
    base_goal.orientation.w = 0.01238;

    youbotBase->publishGoal(base_goal);*/

    if(qrDetector->detect()){

	    if(qrDetector->isQR()){
		ROS_INFO("QR DETECTED!");
		ROS_INFO("The QR pose is: ");
	    	base_goal = qrDetector->getPose();
		ROS_INFO("Position (x, y, z): %e, %e, %e", base_goal.position.x, base_goal.position.y, base_goal.position.z);
		ROS_INFO("Orientation (x, y, z, w): %e, %e, %e, %e", base_goal.orientation.x, base_goal.orientation.y, base_goal.orientation.z, base_goal.orientation.w);
	    	youbotBase->publishGoal(base_goal);
	    }
    
    } else ROS_ERROR("Error with the QR detection.");
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

      ros::spinOnce();  
    }*/
    delete youbotArm;
    youbotArm = 0;
   
    delete youbotBase;
    youbotBase = 0;

    delete qrDetector;
    qrDetector = 0;	
	
    return 0;

   
}

