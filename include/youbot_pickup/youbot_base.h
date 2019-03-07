#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>


class YoubotBase {

	/* Private variables */

	// Publishers 
	ros::Publisher base_pub;

    public:
	
	/* Constructor */
	YoubotBase(ros::NodeHandle& n);

	void publishGoal(geometry_msgs::Pose& p);

};
