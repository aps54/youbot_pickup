#include <youbot_pickup/youbot_arm.h>

YoubotArm::YoubotArm(ros::NodeHandle& n){

  // Publishers
  arm_pub = n.advertise<brics_actuator::JointPositions>("arm_1/arm_controller/position_command", 1);
  gripper_pub = n.advertise<brics_actuator::JointPositions>("arm_1/gripper_controller/position_command", 1);

  // Instance & initialize IK solver       
  pluginlib::ClassLoader<kinematics::KinematicsBase> loader("moveit_core", "kinematics::KinematicsBase");
  try
  {	
     ik_ = loader.createInstance("youbot_arm_kinematics_moveit::KinematicsPlugin");
     ik_->initialize("/robot_description", "arm_1", "arm_link_0", "arm_link_5", 0.01);
     ROS_INFO("IK initialized.");
  }

  catch(pluginlib::PluginlibException& ex)
  {
     ROS_ERROR("Error loading the IK plugin. Error: %s", ex.what());
     exit(0);
  }
}

void YoubotArm::publishArmValues(std::vector<double>& p){
   
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

void YoubotArm::publishGripperValues(double w){

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

void YoubotArm::goHome(){

   std::vector<double> homeValues;
   
   homeValues.push_back(0.11); //A1
   homeValues.push_back(0.11); //A2
   homeValues.push_back(-0.11);//A3
   homeValues.push_back(0.11); //A4
   homeValues.push_back(0.111);//A5
   
   ROS_INFO("Going to home...");

   publishArmValues(homeValues);
   publishGripperValues(0.0);

}

int YoubotArm::moveArm(std::vector<double>& seed, tf::Transform& goal){

    std::vector<double> solution;
    moveit_msgs::MoveItErrorCodes error_code;
    geometry_msgs::Pose goal_pose;

    tf::Vector3& p = goal.getOrigin();
    goal_pose.position.x = p.getX();
    goal_pose.position.y = p.getY();
    goal_pose.position.z = p.getZ();

    tf::Quaternion q = goal.getRotation();

    goal_pose.orientation.w = q.getW();
    goal_pose.orientation.x = q.getX();
    goal_pose.orientation.y = q.getY();
    goal_pose.orientation.z = q.getZ();

    
    ROS_INFO("Goal pose: ");
    std::cout << "Position(x, y, z) = (" <<  goal_pose.position.x << ", " << goal_pose.position.y << ", " << goal_pose.position.z << ")" << std::endl;
    std::cout << "Rotation(x, y, z, w) = (" << goal_pose.orientation.x << ", " << goal_pose.orientation.y << ", " << goal_pose.orientation.z << ", " << goal_pose.orientation.w <<  ")" << std::endl;

    ik_->getPositionIK(goal_pose, seed, solution, error_code);
    if(error_code.val == error_code.SUCCESS){
	ROS_INFO("The solutions are: ");
     	printf("A1. %e; A2. %e; A3. %e; A4. %e; A5. %e;\n", solution[0], solution[1], solution[2], solution[3], solution[4]);
	     
	publishArmValues(solution);
	return 0;						
		
     } else{ std::cout << "Error: " << error_code.val << std::endl;
	return -1;
     }
}
