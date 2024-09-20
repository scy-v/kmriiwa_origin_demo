#include <ros/ros.h>
#include <std_msgs/String.h>
#include <onrobot_rg2ft_msgs/RG2FTCommand.h>
int main(int argc, char** argv)
{
    ros::init(argc, argv, "gripper_rg2_topic_cpp");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<onrobot_rg2ft_msgs::RG2FTCommand>("/command", 20);
    onrobot_rg2ft_msgs::RG2FTCommand msg;
    msg.TargetForce = 200;
    msg.TargetWidth = 700;
    msg.Control = 0x0001;
    while(true)
    {
        ros::Duration(0.1).sleep();
        pub.publish(msg);
        ros::Duration(0.1).sleep();
        if (pub.getNumSubscribers() == 1)
        {
            pub.publish(msg);
            break;
        }
    }
}
