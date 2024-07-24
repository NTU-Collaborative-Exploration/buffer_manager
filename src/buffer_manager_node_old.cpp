#include <ros/ros.h>
#include <ros/console.h>
#include <buffer_manager/buffer_manager.h>

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "buffer_manager");
    ros::NodeHandle priv_nh("~");

    std::string logger_level;
    priv_nh.param<std::string>("logger_level", logger_level, "Info");

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

    BufferManager buffer_manager_node(priv_nh);
    ros::AsyncSpinner spinner(3); // use 2 threads
    spinner.start();

    ros::waitForShutdown();
    return 0;
}