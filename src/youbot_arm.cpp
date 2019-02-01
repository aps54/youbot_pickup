#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/Transform.h>
#include <geometry_msgs/Pose.h>
#include <youbot_arm_kinematics_moveit>

#include <iostream>
#include <vector>

using namespace youbot_arm_kinematics_moveit;

tf::Point in_point;
tf::Transform goal_tf;

geometry_msgs::Pose goal_pose;

boost::shared_ptr<KinematicsPlugin> ik_;

int main(int argc, char** argv){

   if(argc < 4){
	std::cout << "Type a point: x y z" << std::endl;
   }

   ros::init(argc, argv, "youbot_pickup");
   ros::NodeHandle nh("~");

   in_point.setX(argv[1]);
   in_point.setY(argv[2]);
   in_point.setZ(argv[3]);

   goal_tf.setOrigin(in_point);
   goal_tf.setRotation(tf::Quaternion::getIdentity());

   goal_pose.delete();
   goal_pose.position.x = in_point.getX();
   goal_pose.position.y = in_point.getY();
   goal_pose.position.z = in_point.getZ();

   goal_pose.orientation = goal_tf.getRotation();

   // Instance & initialize IK solver
   ik_.reset(new KinematicsPlugin::KinematicsPlugin());
   ik_->initialize("/robot_description", "arm_1", "arm_link_0", "arm_link_5", 0.1);

   std::vector<double> solution;
   moveit_msgs::MoveItErrorCodes error_code;
   
}

