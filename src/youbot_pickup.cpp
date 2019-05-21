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

enum states {
   ready,
   searching,
   navigating,
   take_object
};

int main(int argc, char** argv){
  
    ros::init(argc, argv, "youbot_pickup");
    ros::NodeHandle nh("~"); // Private
    ros::NodeHandle n;

    geometry_msgs::Pose base_goal;
    states robot_state;	

    // TF
    tf::Transform goal_tf;
    tf::Vector3 p;
    tf::Quaternion q;
    tf::TransformListener* tfListener = new tf::TransformListener();
    tf::StampedTransform a0_to_base_link;
    tf::StampedTransform a0_to_a5;
    tf::StampedTransform camera_to_base;
    tf::StampedTransform camera_to_odom;
	
    sleep(2); // For fill TF listener.
	
    try {
	tfListener->lookupTransform("arm_link_0", "base_link", ros::Time(0), a0_to_base_link);
	tfListener->lookupTransform("arm_link_0", "arm_link_5", ros::Time(0), a0_to_a5);
    } catch(tf::LookupException e){
	ROS_ERROR("Error looking the TF. Error: %s", e.what());
    }

   /*-- Initializations --*/ 
	
    YoubotArm* youbotArm = new YoubotArm(n);

    YoubotBase* youbotBase = new YoubotBase(n);

    QRDetector* qrDetector = new QRDetector();

    robot_state = ready;

    while(ros::ok()){

	switch (robot_state){
	   
	   case ready: 

/*  PRUEBAS
    base_goal.position.x = 0.06373;
    base_goal.position.y = 0.84160;
    base_goal.position.z = 0.0;

    base_goal.orientation.x = 0.0;
    base_goal.orientation.y = 0.0;
    base_goal.orientation.z = 0.36485;
    base_goal.orientation.w = 0.01238;

    youbotBase->publishGoal(base_goal);
*/

 	       /*-- QR code detection and pose estimation --*/

	       if(qrDetector->detect()){

		   ROS_INFO("QR DETECTED!");
		   ROS_INFO("The QR pose is: ");
	    	   base_goal = qrDetector->getPose();
		   ROS_INFO("Position (x, y, z): %e, %e, %e", base_goal.position.x, base_goal.position.y, base_goal.position.z);
		   ROS_INFO("Orientation (x, y, z, w): %e, %e, %e, %e", base_goal.orientation.x, base_goal.orientation.y, base_goal.orientation.z, base_goal.orientation.w);

			// Transform the pose to odom frame

		/*try {
		  tfListener->lookupTransform("camera_link", "odom", ros::Time(0), camera_to_odom);

		  geometry_msgs::Transform transform;		

		  p = camera_to_odom.getOrigin();
		  q = camera_to_odom.getRotation();
		  ROS_INFO("The camera to odom transform:");
		  ROS_INFO("Translation (x, y, z): %e, %e, %e", p.getX(), p.getY(), p.getZ());
		  ROS_INFO("Rotation (x, y, z, w): %e, %e, %e, %e", q.getX(), q.getY(), q.getZ(), q.getW());

		  tf::Transform goal_tf;
		  p.setValue(base_goal.position.x, base_goal.position.y, base_goal.position.z);
		  goal_tf.setOrigin(p);
		  q.setValue(base_goal.orientation.x, base_goal.orientation.y, base_goal.orientation.z, base_goal.orientation.w);
		  goal_tf.setRotation(q);
		  goal_tf = goal_tf * camera_to_odom; // Do the transform

		  p = goal_tf.getOrigin();
		  base_goal.position.x = p.getX();
		  base_goal.position.y = p.getY();
		  base_goal.position.z = p.getZ();

		  q = goal_tf.getRotation();
		  base_goal.orientation.x = q.getX();
		  base_goal.orientation.y = q.getY();
		  base_goal.orientation.z = q.getZ();
		  base_goal.orientation.w = q.getW();

		  ROS_INFO("The computed pose will be:");
		  ROS_INFO("Position (x, y, z): %e, %e, %e", base_goal.position.x, base_goal.position.y, base_goal.position.z);
		  ROS_INFO("Orientation (x, y, z, w): %e, %e, %e, %e", base_goal.orientation.x, base_goal.orientation.y, base_goal.orientation.z, base_goal.orientation.w);

    		} catch(tf::LookupException e){
		  ROS_ERROR("Error looking the TF (camera_link --> odom). Error: %s", e.what());
    		}*/

		robot_state = searching;
		
	       } else ROS_ERROR("Error with the QR detection.");

	       break;
	
	   case searching:
		robot_state = navigating;

		youbotBase->publishGoal(base_goal);
		
		break;

	   case navigating:
		robot_state = take_object;

	    	/*-- Pre-grasp position. --*/
	    	ROS_INFO("Going to pre-grasp position...");
	    	youbotArm->goToPregrasp();
	    	youbotArm->openGripper(); // Open the gripper with the move of the arm.				
		
	    	sleep(5); // Wait for the arm to adopt the pose 

	    	/*-- Go to the grasp position. --*/
	    	ROS_INFO("Going to object grasp position...");    	     
	    	youbotArm->goToGrasp();	
	    	sleep(5); // Wait for reseach the goal.    
	
	    	// Grasping the object.
	    	youbotArm->closeGripper();		      

	    	sleep(5); // Wait until the gripper closes.

	    	/*-- Place the object in the plataform. --*/
	    	ROS_INFO("Going to place object position...");		
	    	youbotArm->goToPlace();
	    	sleep(2); // Wait to prevent falls.

	    	youbotArm->openGripper();
	    	sleep(5); // Wait until the gripper is all open.

	    	youbotArm->goHome();

	    	robot_state = ready;
		break;
	}
	ros::spinOnce();  

}
    delete tfListener;
    tfListener = 0;

    delete youbotArm;
    youbotArm = 0;
   
    delete youbotBase;
    youbotBase = 0;

    delete qrDetector;
    qrDetector = 0;	
	
    return 0;

   
}

