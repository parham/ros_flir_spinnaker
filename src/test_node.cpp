
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv) {
    ros::init(argc, argv, "flir_test");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("phm_test", 1000);
    ros::Rate loop_rate(10);
    int count = 0;
    while(ros::ok()) {
        std_msgs::String msg;
        std::stringstream ss;
        ss << "hello world " << count++;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());
        chatter_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
