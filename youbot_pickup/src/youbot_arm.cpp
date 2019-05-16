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

  // Set values for IK seed values. (Obtained experimentaly)
  initSeed.push_back(2.95);
  initSeed.push_back(2.32);
  initSeed.push_back(-2.18);
  initSeed.push_back(3.36);
  initSeed.push_back(3.00);

  graspSeed.push_back(2.95);
  graspSeed.push_back(2.37);
  graspSeed.push_back(-1.64);
  graspSeed.push_back(2.77);
  graspSeed.push_back(3.00);

  placeSeed.push_back(3.09);
  placeSeed.push_back(-0.0);
  placeSeed.push_back(-2.61);
  placeSeed.push_back(0.017);
  placeSeed.push_back(2.92);

  // Set transforms for IK. (Empirically obtained)
  tf::Vector3 t;
  t.setValue(0.341, 0.006, 0.047);
  tf::Quaternion rotation(-0.051, 0.999, -0.000, 0.013);
	
  init_tf.setOrigin(t);  	
  init_tf.setRotation(rotation);

  t.setValue(0.321, 0.005, -0.038);
  rotation.setValue(-0.051, 0.999, -0.000, 0.009);

  grasp_tf.setOrigin(t);
  grasp_tf.setRotation(rotation);

  t.setValue(-0.227, 0.034, 0.100);
  rotation.setValue(0.067, 0.994, 0.005, -0.083);

  place_tf.setOrigin(t);
  place_tf.setRotation(rotation);


  // Set home values
  homeValues.push_back(0.11); //A1
  homeValues.push_back(0.11); //A2
  homeValues.push_back(-0.11);//A3
  homeValues.push_back(0.11); //A4
  homeValues.push_back(0.111);//A5

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

//   ROS_INFO("END publishArmValues");
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

/* Auxiliar functions */

void YoubotArm::goHome(){
   
   ROS_INFO("Going to home...");

   publishArmValues(homeValues);
   publishGripperValues(0.0);

}

tf::Transform YoubotArm::initialPose(){

  return init_tf;
}

void YoubotArm::openGripper(){

    publishGripperValues(0.0115);

}

void YoubotArm::closeGripper(){

    publishGripperValues(0.0);

}

void YoubotArm::goToPregrasp(){
    
    moveArm(initSeed, init_tf);

}

void YoubotArm::goToGrasp(){

    moveArm(graspSeed, grasp_tf);

}

void YoubotArm::goToPlace(){

    moveArm(placeSeed, place_tf);

}
