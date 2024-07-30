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
#include <sensor_msgs/Image.h>
#include "dislam_msgs/SubMap.h"


template <typename MSG_T>
class BufferManager
{
private:
    ros::NodeHandle nh_;

    /* node parameters */
    bool is_respawn_;
    std::string msg_tp_in_, msg_tp_out_;
    std::string msg_backup_filename_;
    std::string last_sent_id_filename_;
    int buffer_size_limit_;
    int timeout_limit_ms_; // unit: ms
    std::string msg_ns_;

    /* publisher */
    ros::Publisher msg_pub_;

    /* subscriber */
    ros::Subscriber msg_sub_;

    /* timer */
    ros::Timer ping_client_timer;
    ros::Timer msg_pub_timer;

    /* msg buffer */
    std::queue<MSG_T> msg_buffer_;

    /* thread mutex */
    std::mutex msg_buffer_mutex_;

    /* tcp connection params */
    std::string ip_to_query_;
    boost::asio::io_service io_service_;
    std::chrono::steady_clock::time_point last_connected_time_;
    bool is_connected_;

    /* counter */
    uint32_t last_sent_id_;
    uint32_t msg_backup_count_;

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
    
    /**
     * @brief Load the id of the last sent msg from file on disk.
     * @details update the last_sent_id_
     */ 
    void loadLastSentId();

    /**
     * @brief Save msg into the file on disk
     * @details Saved into a binary file by 'appending'
     */
    void saveMsgToDisk(const MSG_T &msg);

    void removeOutdateMsgFromDisk();

    void recoverMsgBuffer();

    void msgCallback(const typename MSG_T::ConstPtr &msg);

    void msgPublish(const ros::TimerEvent &);

    void pingClient(const ros::TimerEvent &);
};

template <typename MSG_T>
BufferManager<MSG_T>::BufferManager(ros::NodeHandle priv_nh)
{
    // get params from launch file
    priv_nh.param<std::string>("msg_ns_", msg_ns_, "");
    priv_nh.param<std::string>("ip_to_query_", ip_to_query_, "192.168.1.100");
    priv_nh.param<std::string>("msg_tp_in_", msg_tp_in_, "/robot_1/submap_raw");
    priv_nh.param<std::string>("msg_tp_out_", msg_tp_out_, "/robot_1/submap");
    priv_nh.param<std::string>("msg_backup_filename_", msg_backup_filename_, "submap_backup.dat");
    priv_nh.param<std::string>("last_sent_id_filename_", last_sent_id_filename_, "last_sent_submap_id.dat");
    priv_nh.param("buffer_size_limit_", buffer_size_limit_, 20);
    priv_nh.param("timeout_limit_ms_", timeout_limit_ms_, 5000);

    ROS_INFO("msg_ns_: %s", msg_ns_.c_str());
    ROS_INFO("ip_to_query_: %s", ip_to_query_.c_str());
    ROS_INFO("msg_tp_in_: %s", msg_tp_in_.c_str());
    ROS_INFO("msg_tp_out_: %s", msg_tp_out_.c_str());
    ROS_INFO("msg_backup_filename_: %s", msg_backup_filename_.c_str());
    ROS_INFO("last_sent_id_filename_: %s", last_sent_id_filename_.c_str());
    ROS_INFO("buffer_size_limit: %d", buffer_size_limit_);
    ROS_INFO("timeout_limit_ms_: %d", timeout_limit_ms_);

    // check if this node respawned
    ros::param::get("/" + msg_ns_ + "/is_respawn", is_respawn_);
    ROS_INFO("is_respawn_: %d", is_respawn_);
    if(is_respawn_){
        ROS_INFO("[%s]recover buffer from file", msg_ns_.c_str());
        loadLastSentId();
        recoverMsgBuffer();
    }
    else{
        std::ofstream outfile(msg_backup_filename_, std::ios::trunc | std::ios::binary);
        ros::param::set("/is_respawn", true);
        is_respawn_ = true;
    }

    // initialization
    last_connected_time_ = std::chrono::steady_clock::now();
    msg_sub_ = priv_nh.subscribe(msg_tp_in_, 100, &BufferManager::msgCallback, this);
    ping_client_timer = priv_nh.createTimer(ros::Duration(0.1), &BufferManager::pingClient, this);
    msg_pub_timer = priv_nh.createTimer(ros::Duration(0.1), &BufferManager::msgPublish, this);
    msg_pub_ = nh_.advertise<MSG_T>(msg_tp_out_, 100);

    is_connected_ = false;
    last_sent_id_ = 0;
    msg_backup_count_ = 0;
}

template <typename MSG_T>
BufferManager<MSG_T>::~BufferManager()
{
    std::lock_guard<std::mutex> lock(msg_buffer_mutex_);
    msg_buffer_.emplace();
}

template <typename MSG_T>
void BufferManager<MSG_T>::saveLastSentId()
{
    std::ofstream outfile(last_sent_id_filename_, std::ios::binary);
    if(outfile.is_open())
    {
        outfile.write(reinterpret_cast<const char *>(&last_sent_id_), sizeof(last_sent_id_));
        outfile.close();
    }
}

template <typename MSG_T>
void BufferManager<MSG_T>::loadLastSentId()
{
    std::ifstream infile(last_sent_id_filename_, std::ios::binary);
    if (infile.is_open())
    {
        infile.read(reinterpret_cast<char *>(&last_sent_id_), sizeof(last_sent_id_));
        infile.close();
    }
    ROS_DEBUG("[%s/loadLastSentId] last_sent_id: %d", msg_ns_.c_str(), last_sent_id_);
}

template <typename MSG_T>
void BufferManager<MSG_T>::saveMsgToDisk(const MSG_T &msg)
{
    std::ofstream outfile(msg_backup_filename_, std::ios::binary | std::ios::app);
    if(outfile.is_open())
    {
        uint32_t msg_size = ros::serialization::serializationLength(msg);
        outfile.write(reinterpret_cast<const char *>(&msg_size), sizeof(msg_size));

        std::vector<uint8_t> buffer(msg_size);
        ros::serialization::OStream stream(buffer.data(), msg_size);
        ros::serialization::serialize<MSG_T>(stream, msg);
        outfile.write(reinterpret_cast<const char *>(buffer.data()), msg_size);

        outfile.close();
        msg_backup_count_++;
    }
    ROS_DEBUG("[%s/saveMsgToDisk] saved %d msgs", msg_ns_.c_str(), msg_backup_count_);
}

template <typename MSG_T>
void BufferManager<MSG_T>::removeOutdateMsgFromDisk()
{
    if (msg_backup_count_ < buffer_size_limit_ || last_sent_id_ == 0)
        return;

    std::ifstream infile(msg_backup_filename_, std::ios::binary);
    std::string temp_filename = msg_backup_filename_ + ".temp";
    std::ofstream temp_file(temp_filename, std::ios::binary);
    if(infile.is_open() && temp_file.is_open())
    {
        uint32_t current_id = 0;
        while (!infile.eof())
        {
            uint32_t msg_size;
            infile.read(reinterpret_cast<char *>(&msg_size), sizeof(msg_size));
            if(infile.eof()) break;
            if(current_id >= last_sent_id_)
            {
                std::vector<uint8_t> buffer(msg_size);
                infile.read(reinterpret_cast<char*>(buffer.data()), msg_size);
                temp_file.write(reinterpret_cast<const char *>(&msg_size), sizeof(msg_size));
                temp_file.write(reinterpret_cast<const char *>(buffer.data()), msg_size);
            }
            else
            {
                infile.seekg(msg_size, std::ios::cur);
            }
            ++current_id;
        }
        infile.close();
        temp_file.close();
        remove(msg_backup_filename_.c_str());
        rename(temp_filename.c_str(), msg_backup_filename_.c_str());
        msg_backup_count_ = msg_backup_count_ - last_sent_id_;
        last_sent_id_ = 0;
        saveLastSentId();

        ROS_DEBUG("[%s/removeHistory] update submap_backup_count: %d", msg_ns_.c_str(), msg_backup_count_);
    }
}

template <typename MSG_T>
void BufferManager<MSG_T>::recoverMsgBuffer()
{
    std::lock_guard<std::mutex> lock(msg_buffer_mutex_);
    std::ifstream infile(msg_backup_filename_, std::ios::binary);
    if(infile.is_open())
    {
        uint32_t current_id = 0;
        while (!infile.eof())
        {
            uint32_t msg_size;
            infile.read(reinterpret_cast<char *>(&msg_size), sizeof(msg_size));
            if(infile.eof()) break;
            if(current_id > last_sent_id_)
            {
                std::vector<uint8_t> buffer(msg_size);
                infile.read(reinterpret_cast<char*>(buffer.data()), msg_size);
                MSG_T msg;
                ros::serialization::IStream stream(buffer.data(), msg_size);
                ros::serialization::deserialize(stream, msg);
                if(msg_buffer_.size() >= buffer_size_limit_)
                {
                    msg_buffer_.pop();
                }
                msg_buffer_.push(msg);
            }
            else
            {
                infile.seekg(msg_size, std::ios::cur);
            }
            ++current_id;
            ++msg_backup_count_;
        }

        ROS_DEBUG("[%s/recover] submap save_count: %d; sent_id: %d; buffer size: %d", msg_ns_.c_str(), msg_backup_count_, last_sent_id_, msg_buffer_.size());
    }
    infile.close();
}

template <typename MSG_T>
void BufferManager<MSG_T>::msgCallback(const typename MSG_T::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(msg_buffer_mutex_);
    if(msg_buffer_.size() >= buffer_size_limit_){
        msg_buffer_.pop();
    }
    msg_buffer_.push(*msg);
    // ROS_DEBUG("[%s/msgCallback] recieved msg width: %d", msg_ns_.c_str(), msg->submap.width);
    ROS_DEBUG("[%s/msgCallback] push! buffer size: %d", msg_ns_.c_str(), msg_buffer_.size());
    saveMsgToDisk(*msg);
}

template <typename MSG_T>
void BufferManager<MSG_T>::msgPublish(const ros::TimerEvent &)
{
    std::lock_guard<std::mutex> lock(msg_buffer_mutex_);
    if(is_connected_ && msg_pub_.getNumSubscribers() > 0 && !msg_buffer_.empty())
    {
        ROS_DEBUG("[%s/msgPub]backup count: %d; last sent: %d", msg_ns_.c_str(), msg_backup_count_, last_sent_id_);
        if (msg_buffer_.size() >= buffer_size_limit_ && msg_backup_count_ > last_sent_id_ + buffer_size_limit_)
        { // pub from file
            ROS_DEBUG("[%s/msgPub] publish from file", msg_ns_.c_str());
            std::ifstream infile(msg_backup_filename_, std::ios::binary);
            if(infile.is_open())
            {
                uint32_t current_id = 0;
                while (!infile.eof())
                {
                    uint32_t msg_size;
                    infile.read(reinterpret_cast<char *>(&msg_size), sizeof(msg_size));
                    // ROS_DEBUG("[%s/msgPub] current_id: %d; read msg_size: %d", msg_ns_.c_str(), current_id, msg_size);

                    if (infile.eof())
                        break;
                    if (current_id >= last_sent_id_)
                    {
                        std::vector<uint8_t> buffer(msg_size);
                        infile.read(reinterpret_cast<char *>(buffer.data()), msg_size);
                        MSG_T msg;
                        ros::serialization::IStream stream(buffer.data(), msg_size);
                        ros::serialization::deserialize<MSG_T>(stream, msg);
                        msg_pub_.publish(msg);
                        ++last_sent_id_;

                        // ROS_DEBUG("[%s/msgPub] file send submap size: %d; buffer front submap size: %d", msg_ns_.c_str(), msg.submap.width, msg_buffer_.front().submap.width);
                        break;
                    }
                    else
                    {
                        infile.seekg(msg_size, std::ios::cur);
                    }
                    ++current_id;
                }
            }
            infile.close();
        }
        else
        { // pub from buffer
            ROS_DEBUG("[%s/msgPub] publish from buffer", msg_ns_.c_str());
            MSG_T &msg = msg_buffer_.front();
            msg_pub_.publish(msg);
            msg_buffer_.pop();
            ++last_sent_id_;
            ROS_DEBUG("[%s/msgPub] buffer size after: %d", msg_ns_.c_str(), msg_buffer_.size());
        }
        saveLastSentId();
    }
    removeOutdateMsgFromDisk();
}

template <typename MSG_T>
void BufferManager<MSG_T>::pingClient(const ros::TimerEvent &)
{
    try
    {
        boost::asio::ip::tcp::socket socket(io_service_);
        boost::asio::ip::tcp::resolver resolver(io_service_);
        boost::asio::ip::tcp::resolver::query query(boost::asio::ip::tcp::v4(), ip_to_query_, "11311"); // port with ROS_MASTER
        boost::asio::ip::tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
        boost::asio::connect(socket, endpoint_iterator);
        is_connected_ = true;
        last_connected_time_ = std::chrono::steady_clock::now();
        socket.close();
        ROS_DEBUG("[%s]!!Connected with %s", msg_ns_.c_str(), ip_to_query_.c_str());
    }
    catch (const boost::system::system_error& e) {
        is_connected_ = false;
        ROS_WARN("[%s] Connection failed: %s", msg_ns_.c_str(), e.what());
    }
    // catch (...)
    // {
    //     is_connected_ = false;
    //     ROS_WARN("[%s]!!Disonnected with %s", msg_ns_.c_str(), ip_to_query_.c_str());
    // }

    // check timeout
    if (!is_connected_) 
    {
        auto current_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_connected_time_);
        if (duration.count() > timeout_limit_ms_) {
            ROS_WARN("[%s]TIME OUT! time: %fs", msg_ns_.c_str(), float(duration.count()/1000.0f));
        }
    }
}

#endif