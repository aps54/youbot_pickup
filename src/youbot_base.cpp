#include <youbot_pickup/youbot_base.h>

base_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
