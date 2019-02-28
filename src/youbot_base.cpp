#include <youbot_pickup/youbot_base.h>

/* Constructor */
YoubotBase::YoubotBase(ros::NodeHandle& n){

    // Publishers
    base_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);

}
