#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <string>

using namespace std;

ros::Publisher test_msg_pub, test_msg2_pub;
uint16_t num = 0;

void pubTestMsg(const ros::TimerEvent &)
{
    std_msgs::UInt16 msg;
    msg.data = num;
    test_msg_pub.publish(msg);
    num = num + 1;
}

void pubTestMsg2(const ros::TimerEvent &)
{
    std_msgs::String msg;
    msg.data = "Hello " + to_string(num);
    test_msg2_pub.publish(msg);
    ++num;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_talker");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");

    // test_msg_pub = nh.advertise<std_msgs::UInt16>("/test_msg", 100);
    test_msg2_pub = nh.advertise<std_msgs::String>("/test_msg", 100);

    // ros::Timer test_msg_timer = nh.createTimer(ros::Duration(1.0), pubTestMsg);
    ros::Timer test_msg_timer = nh.createTimer(ros::Duration(1.0), pubTestMsg2);

    ros::spin();

    return 0;
}