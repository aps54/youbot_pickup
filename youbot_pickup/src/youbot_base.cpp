#include <youbot_pickup/youbot_base.h>

/* Constructor */
YoubotBase::YoubotBase(ros::NodeHandle& n){

    // Publishers
    base_pub = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);

}

void YoubotBase::publishGoal(geometry_msgs::Pose& p){

	geometry_msgs::PoseStamped goal;
	
	goal.header.stamp = ros::Time::now();
	goal.header.frame_id = "odom";

	goal.pose = p;

	base_pub.publish(goal);

}
