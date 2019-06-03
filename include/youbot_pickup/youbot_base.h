#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>

#include <tgmath.h> // atan2 & fabs
#include <stdlib.h> // abs

class YoubotBase {

	/* Private variables */

	// Publishers 
	ros::Publisher base_pub, vel_pub;

	// Suscriber
	ros::Subscriber odom_sub;

	// Global variables
	double x;
	double y;
	double theta;

	bool odom_rec;

	/* Private functions */

	// Taking a quaternion in x, y, z, w axis transform to euler angles (roll, pitch, yaw)
	void toEulerAngle(const geometry_msgs::Quaternion& q, double& roll, double& pitch, double& yaw);

	// When a new odom message arrives, do the transformation to euler angles and take the needed info
	// in the global variables.
	void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);

    public:
	
	/* Constructor */
	YoubotBase(ros::NodeHandle& n);

	bool publishGoal(geometry_msgs::Pose& p);

	/* Auxiliar function */
	void initialPose(double& x, double& y, double& z);

};
