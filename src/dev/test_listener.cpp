#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <string>

using namespace std;

void testMsgCallback(const std_msgs::UInt16::ConstPtr& msg)
{
    cout << "[listener] heared msg: " << msg->data << endl;
}

void testMsgCallback2(const std_msgs::String::ConstPtr& msg)
{
    cout << "[listener] heared msg: " << msg->data << endl;
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "test_listener");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");

    // ros::Subscriber test_msg_listener = nh.subscribe("/buffer_test_node/test_msg_out", 100, testMsgCallback);
    ros::Subscriber test_msg_listener = nh.subscribe("/buffer_test_node/test_msg_out", 100, testMsgCallback2);

    ros::spin();

    return 0;
}