#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/Transform.h>
#include <geometry_msgs/Pose.h>
#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointValue.h>
#include <youbot_arm_kinematics_moveit>

#include <iostream>
#include <sstream>
#include <vector>

using namespace youbot_arm_kinematics_moveit;

tf::Point in_point;
tf::Transform goal_tf;

geometry_msgs::Pose goal_pose;
//geometry_msgs::Pose aprx_pose;

brics_actuator::JointPosition pos;

std::vector<double> seed;
std::vector<brics_actuator::JointValue> armJointPositions;
std::vector<brics_actuator::JointValue> gripperJointPositions;

boost::shared_ptr<KinematicsPlugin> ik_;

// Publishers 
ros::Publisher arm_pub, gripper_pub;

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

   // Publishers
   arm_pub = n.advertise<brics_actuator::JointPositions>("arm_1/arm_controller/position_command", 1);
   gripper_pub = n.advertise<brics_actuator::JointPositions>("arm_1/gripper_controller/position_command", 1);

   // Instance & initialize IK solver
   ik_.reset(new KinematicsPlugin::KinematicsPlugin());
   ik_->initialize("/robot_description", "arm_1", "arm_link_0", "arm_link_5", 0.1);

   std::vector<double> solution;
   moveit_msgs::MoveItErrorCodes error_code;
   
   // Seed values. Obtained with rviz (moveit plugin).
   seed.push_back(2.83552);
   seed.push_back(2.20516);
   seed.push_back(-1.77035);
   seed.push_back(3.00807);
   seed.push_back(0.10603);

   ik_->getPositionIK(goal_pose, seed, solution, error_code);

   publishArmValues(solution);
   publishGripperValues(0.0115);
}

void publishArmValues(std::vector<double> p){
   
   stringstream jointName;
   armJointPositions.resize(p.size());

   for (unsigned int i=0; i < p.size(); i++){
	jointName.str("");
	jointName << "arm_joint_" << (i+1);
	armJointPositions[i].joint_uri = jointName.str();
	armJointPositions[i].unit ? boost::units::to_string(boost::units::si:radians);
	armJointPositions[i].value = p[i];
   }

   pos.positions = armJointPositions;
   arm_pub.publish(pos);
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
