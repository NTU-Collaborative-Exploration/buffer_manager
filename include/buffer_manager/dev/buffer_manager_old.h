#ifndef buffer_manager_H
#define buffer_manager_H

#include <ros/ros.h>
#include <ros/serialization.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <string>
#include <queue>
#include <thread>
#include <mutex>
#include <fstream>
#include <chrono>

#include <sensor_msgs/PointCloud2.h>
#include "dislam_msgs/SubMap.h"

class BufferManager
{
private:
    ros::NodeHandle nh_;

    /* node parameters */
    bool is_respawn_;
    std::string submap_tp_in_, submap_tp_out_;
    std::string colormap_tp_in_, colormap_tp_out_;
    std::string submap_backup_filename_;
    std::string colormap_backup_filename_;
    std::string last_sent_id_filename_;
    std::string last_sent_submap_id_filename_;
    std::string last_sent_colormap_id_filename_;
    int buffer_size_limit_;
    int timeout_limit_ms_; // unit: ms

    /* publisher */
    ros::Publisher submap_pub_;
    ros::Publisher colormap_pub_;

    /* subscriber */
    ros::Subscriber submap_sub_;
    ros::Subscriber colormap_sub_;

    /* timer */
    ros::Timer ping_client_timer;
    ros::Timer submap_pub_timer;
    ros::Timer colormap_pub_timer;

    /* msg buffer */
    std::queue<dislam_msgs::SubMap> submap_buffer_;
    std::queue<sensor_msgs::PointCloud2> colormap_buffer_;

    /* thread mutex */
    std::mutex submap_buffer_mutex_;
    std::mutex colormap_buffer_mutex_;

    /* tcp connection params */
    std::string ip_to_query_;
    boost::asio::io_service io_service_;
    std::chrono::steady_clock::time_point last_connected_time_;
    bool is_connected_;

    /* counter */
    uint32_t last_sent_submap_id_;
    uint32_t submap_backup_count_;
    uint32_t last_sent_colormap_id_;
    uint32_t colormap_backup_count_;

public:
    /**
     * @brief Initiates and get some variables from the launch file under current ROS core.
     * @details When constructed, some prepration is done for further calculation.
     */  
    BufferManager(ros::NodeHandle priv_nh);

    /**
     * @brief Destructor.
     * @details When deconstructed, some memories should be released.
     */  
    ~BufferManager();

    /**
     * @brief Save the id of the last sent msg into file on disk.
     * @details Saved as a binary file
     */ 
    void saveLastSentId();
    void saveLastSentId(std::string &filename, uint32_t &last_sent_msg_id);
    
    /**
     * @brief Load the id of the last sent msg from file on disk.
     * @details update the last_sent_id_
     */ 
    void loadLastSentId();

    /**
     * @brief Save msg into the file on disk
     * @details Saved into a binary file by 'appending'
     */
    template <typename MSG_T>
    void saveMsgToDisk(const MSG_T &msg, std::string &filename, uint32_t &msg_backup_count);
    void saveSubmapMsgToDisk(const dislam_msgs::SubMap &msg, std::string &filename, uint32_t& msg_backup_count);
    void saveColormapMsgToDisk(const sensor_msgs::PointCloud2 &msg, std::string &filename, uint32_t& msg_backup_count);

    void removeOutdateMsgFromDisk(std::string &filename, uint32_t& msg_backup_count, uint32_t& last_sent_msg_id);

    void recoverMsgBuffer();

    void submapMsgCallback(const dislam_msgs::SubMap::ConstPtr &msg);

    void submapMsgPublish(const ros::TimerEvent &);

    void colormapMsgCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);

    void colormapMsgPublish(const ros::TimerEvent &);

    void pingClient(const ros::TimerEvent &);
};





#endif