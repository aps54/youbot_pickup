#include <youbot_pickup/youbot_base.h>

/* Constructor */
YoubotBase::YoubotBase(ros::NodeHandle& n){

    // Global variables
    x = 0.0;
    y = 0.0; 
    theta = 0.0;
    odom_rec = false;

    // Publishers
    //base_pub = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1); descomment for use with move_base module
    vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1); // comment this line if you'll use move_base module

    // Subscriber
    odom_sub = n.subscribe("odom", 1000, &YoubotBase::odomCallback, this);
    
}

// Code from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
void YoubotBase::toEulerAngle(const geometry_msgs::Quaternion& q, double& roll, double& pitch, double& yaw)
{
	// roll (x-axis rotation)
	double sinr_cosp = +2.0 * (q.w * q.x + q.y * q.z);
	double cosr_cosp = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
	roll = atan2(sinr_cosp, cosr_cosp);

	// pitch (y-axis rotation)
	double sinp = +2.0 * (q.w * q.y - q.z * q.x);
	if (fabs(sinp) >= 1)
		pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		pitch = asin(sinp);

	// yaw (z-axis rotation)
	double siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
	double cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);  
	yaw = atan2(siny_cosp, cosy_cosp);
}

void YoubotBase::odomCallback(const nav_msgs::Odometry::ConstPtr& odom){

	x = odom->pose.pose.position.x;
	y = odom->pose.pose.position.y;

	geometry_msgs::Quaternion rot = odom->pose.pose.orientation;

	double roll, pitch;
	toEulerAngle(rot, roll, pitch, theta);
	odom_rec = true;	
}

bool YoubotBase::publishGoal(geometry_msgs::Pose& p){

   /* For using with move_base module this is enought

	geometry_msgs::PoseStamped goal;
	
	goal.header.stamp = ros::Time::now();
	goal.header.frame_id = "odom";

	goal.pose = p;

	base_pub.publish(goal);
   */

    /* Expecific use for this application */

	geometry_msgs::Twist speed;
	double x_, y_, angle_to_goal; // Distances
	double t;		      // Time

	if(odom_rec){ // Ensure that odom has been received.

	   x_ = p.position.x - x;
	   y_ = p.position.y - y;

	   odom_rec = false; // For wait the next odom.

	   std::cout << "X distancia: " << x_ << "  Y distancia: " << y_ << "  Rotación: " << theta << std::endl;

	   angle_to_goal = atan2(x_,y_);

	   std::cout << "Distancia rotación: " << angle_to_goal << std::endl;

	   /*if (abs(angle_to_goal - theta) > 0.1){ // Rotate to the goal
		speed.linear.x = 0.0;
		speed.linear.y = 0.0;
        	speed.angular.z = 0.3;
		t = abs(angle_to_goal - theta) / 0.3;
	   } else {*/
		speed.linear.x = 0.5;
		speed.linear.y = 0.0;
        	speed.angular.z = 0.0;
		if((fabs(x_) - 0.105) <= 0) {
			t = 0.001;
			t = t * 1000000; // convert to microseconds
			speed.linear.x = - 0.5;
		}else {
			t = (fabs(x_) - 0.105) / 0.5; // calculate the time (seconds)
			t = t * 1000000; // convert to microseconds
		}
	   //}
 	
	   vel_pub.publish(speed);

	   std::cout << "*Snooze* I'm sleepy, i'll come back in " << t << " microseconds." << std::endl; 

	   usleep(t);

	   speed.linear.x = 0.0;
	   speed.linear.y = 0.0;
	   speed.angular.z = 0.0;

	   vel_pub.publish(speed);

	   std::cout << "OMG I think I'm a sleepwalker..." << std::endl;
		
	   return true;

	}
	
	/* Only for security reasons */
	speed.linear.x = 0.0;
	speed.linear.y = 0.0;
	speed.angular.z = 0.0;

	vel_pub.publish(speed);
	return false;
}
