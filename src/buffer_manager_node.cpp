#include <ros/ros.h>
#include <ros/console.h>
#include <buffer_manager/buffer_manager.h>
#include <buffer_manager/node_util.h>

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "buffer_manager");
    ros::NodeHandle priv_nh("~");

    std::string logger_level;
    std::string msg_ns;
    priv_nh.param<std::string>("logger_level", logger_level, "Info");
    priv_nh.param<std::string>("msg_ns_", msg_ns, "submap");

    if(logger_level == "Info"){
        // this package is still in development -- start wil debugging enabled
        if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info)) {
            ros::console::notifyLoggerLevelsChanged();
        }
    }else if(logger_level == "Debug"){
        // this package is still in development -- start wil debugging enabled
        if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
            ros::console::notifyLoggerLevelsChanged();
        }
    }else if(logger_level == "Error"){
        // this package is still in development -- start wil debugging enabled
        if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Error)) {
            ros::console::notifyLoggerLevelsChanged();
        }
    }else if(logger_level == "Warn"){
        // this package is still in development -- start wil debugging enabled
        if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn)) {
            ros::console::notifyLoggerLevelsChanged();
        }
    }

    auto bufferManager = createBufferManager(msg_ns, priv_nh);
    if (!bufferManager) {
        ROS_ERROR("[%s]!!!Invalid msg_ns", priv_nh.getNamespace().c_str());
        return 0;
    }

    // BufferManager<dislam_msgs::SubMap> buffer_manager_node(priv_nh);
    ros::AsyncSpinner spinner(0); // use all threads
    spinner.start();
    ros::waitForShutdown();
    return 0;
}