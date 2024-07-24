#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <string>
#include <queue>
#include <thread>
#include <mutex>
#include <fstream>
#include <chrono>

using boost::asio::ip::tcp;
using namespace std;

queue<std_msgs::String> testmsg_buffer;
mutex testmsg_buffer_mutex;

ros::Publisher test_msg_pub;

bool is_respawn = false;
bool is_connected = false;
boost::asio::io_service io_service;

string test_msg_backup_filename;
string last_sent_filename;

std::chrono::steady_clock::time_point last_connected_time;
const int buffer_size_limit = 10;
const int timeout_limit = 10;

uint32_t last_sent_test_msg_id = 0;
uint32_t test_msg_backup_count = 0;

void saveLastSentId()
{
    ofstream outfile(last_sent_filename, ios::binary);
    if(outfile.is_open())
    {
        outfile.write(reinterpret_cast<const char *>(&last_sent_test_msg_id), sizeof(last_sent_test_msg_id));
        outfile.close();
    }
}

void loadLastSentId()
{
    ifstream infile(last_sent_filename, ios::binary);
    if (infile.is_open())
    {
        infile.read(reinterpret_cast<char *>(&last_sent_test_msg_id), sizeof(last_sent_test_msg_id));
        cout << "[loadLastSentId] " << last_sent_test_msg_id << endl;
        infile.close();
    }
}

void saveTestMsgToDisk(const std_msgs::String& msg)
{
    ofstream outfile(test_msg_backup_filename, ios::binary | ios::app);
    // ofstream outfile(test_msg_backup_filename, ios::app);
    if(outfile.is_open())
    {
        uint32_t msg_size = ros::serialization::serializationLength(msg);
        outfile.write(reinterpret_cast<const char *>(&msg_size), sizeof(msg_size));
        // cout << "msg_size: " << msg_size << endl;
        // cout << "sizeof(msg_size) = " << sizeof(msg_size) << endl;
        // boost::shared_array<uint8_t> buffer(new uint8_t[msg_size]);
        vector<uint8_t> buffer(msg_size);
        // ros::serialization::OStream stream(buffer.get(), msg_size);
        ros::serialization::OStream stream(buffer.data(), msg_size);
        ros::serialization::serialize<std_msgs::String>(stream, msg);
        // outfile.write(reinterpret_cast<const char *>(buffer.get()), msg_size);
        outfile.write(reinterpret_cast<const char *>(buffer.data()), msg_size);
        outfile.close();
        // cout << "sizeof(msg):" << sizeof(msg) << endl;
        // cout << "sizeof(buffer):" << sizeof(buffer) << endl;

        // printf("[");
        // for (int i = 0; i < msg_size; i++)
        // {
        //     printf("%x ", buffer[i]);
        // }
        // printf("]\n");
        test_msg_backup_count++;
    }
    cout << "[saveTestMsgToDisk] saved " << test_msg_backup_count << " msgs" << endl;
}

void removeHistoryTestMsgFromDisk()
{
    if(test_msg_backup_count <= buffer_size_limit)
        return;

    cout << "[removeHistory] enter" << endl;
    ifstream infile(test_msg_backup_filename, ios::binary);
    string temp_test_msg_backup_filename = test_msg_backup_filename + ".temp";
    ofstream temp_file(temp_test_msg_backup_filename, ios::binary);
    if(infile.is_open() && temp_file.is_open())
    {
        uint32_t current_id = 0;
        while (!infile.eof())
        {
            uint32_t msg_size;
            infile.read(reinterpret_cast<char *>(&msg_size), sizeof(msg_size));
            if(infile.eof()) break;
            if(current_id >= last_sent_test_msg_id)
            {
                vector<uint8_t> buffer(msg_size);
                infile.read(reinterpret_cast<char*>(buffer.data()), msg_size);
                temp_file.write(reinterpret_cast<const char *>(&msg_size), sizeof(msg_size));
                temp_file.write(reinterpret_cast<const char *>(buffer.data()), msg_size);
            }
            else
            {
                infile.seekg(msg_size, ios::cur);
            }
            ++current_id;
        }
        infile.close();
        temp_file.close();
        remove(test_msg_backup_filename.c_str());
        rename(temp_test_msg_backup_filename.c_str(), test_msg_backup_filename.c_str());
        test_msg_backup_count = test_msg_backup_count - last_sent_test_msg_id;
        last_sent_test_msg_id = 0;
        saveLastSentId();
        cout << "[removeHistory] update test_msg_backup_count = " << test_msg_backup_count << endl;
    }
}

void testmsgCallback(const std_msgs::String::ConstPtr& msg)
{
    // cout << "[testmsgCallback] recieved! buffer size1: " << testmsg_buffer.size() << endl;
    lock_guard<mutex> lock(testmsg_buffer_mutex);
    if(testmsg_buffer.size() >= buffer_size_limit){
        // cout << "[testmsgCallback] buffer reach limit!" << endl;
        testmsg_buffer.pop();
    }
    testmsg_buffer.push(*msg);
    cout << "[testmsgCallback] sub msg: " << msg->data << endl;
    saveTestMsgToDisk(*msg);
    cout << "[testmsgCallback] push! buffer size2: " << testmsg_buffer.size() << endl;
}

void recoverTestMsgBuffer()
{
    ifstream infile(test_msg_backup_filename, ios::binary);
    if(infile.is_open())
    {
        uint32_t current_id = 0;
        while (!infile.eof())
        {
            uint32_t msg_size;
            infile.read(reinterpret_cast<char *>(&msg_size), sizeof(msg_size));
            if(infile.eof()) break;
            if(current_id > last_sent_test_msg_id)
            {
                vector<uint8_t> buffer(msg_size);
                infile.read(reinterpret_cast<char*>(buffer.data()), msg_size);
                std_msgs::String msg;
                ros::serialization::IStream stream(buffer.data(), msg_size);
                ros::serialization::deserialize(stream, msg);
                if(testmsg_buffer.size() >= buffer_size_limit)
                {
                    testmsg_buffer.pop();
                }
                testmsg_buffer.push(msg);
            }
            else
            {
                infile.seekg(msg_size, ios::cur);
            }
            ++current_id;
            ++test_msg_backup_count;
        }
        cout << "[recover] save_count: " << test_msg_backup_count << " sent_id: " << last_sent_test_msg_id << endl;
        cout << "[recover] buffer size: " << testmsg_buffer.size() << endl;
    }
    infile.close();
}

void sendTestMsg(const ros::TimerEvent&)
{
    // cout << "[sendTestMsg] in" << endl;
    lock_guard<mutex> lock(testmsg_buffer_mutex);
    if(is_connected && test_msg_pub.getNumSubscribers() > 0 && !testmsg_buffer.empty())
    {
        cout << "lr: " << test_msg_backup_count << "  " << "ls: " << last_sent_test_msg_id << endl;
        if(testmsg_buffer.size() >= buffer_size_limit && test_msg_backup_count > last_sent_test_msg_id + buffer_size_limit)
        { // pub from file
            cout << "pub from file" << endl;
            ifstream infile(test_msg_backup_filename, ios::binary);
            if(infile.is_open())
            {
                uint32_t current_id = 0;
                while (!infile.eof())
                {
                    cout << "current_id: " << current_id;
                    uint32_t msg_size;
                    infile.read(reinterpret_cast<char *>(&msg_size), sizeof(msg_size));
                    cout << "; read msg_size: " << msg_size << "  " << endl;
                    if(infile.eof()) break;
                    if (current_id >= last_sent_test_msg_id)
                    {
                        vector<uint8_t> buffer(msg_size);
                        infile.read(reinterpret_cast<char *>(buffer.data()), msg_size);
                        std_msgs::String msg;
                        ros::serialization::IStream stream(buffer.data(), msg_size);
                        ros::serialization::deserialize<std_msgs::String>(stream, msg);
                        cout << "; msg: " << msg.data << endl;
                        for (int i = 0; i < msg_size; i++)
                            printf("%x", buffer[i]);
                        printf("\n");
                        test_msg_pub.publish(msg);
                        ++last_sent_test_msg_id;
                        cout << "[sendTestMsg] file send msg: " << msg.data << endl;
                        cout << "[sendTestMsg] buffer front msg: " << testmsg_buffer.front().data << endl;
                        break;
                    }
                    else
                    {
                        infile.seekg(msg_size, ios::cur);
                    }
                    ++current_id;
                }
            }
            infile.close();
        }
        else
        { // pub from buffer
            cout << "[sendTestMsg] publish. buffer size1:" << testmsg_buffer.size() << endl;
            std_msgs::String &msg = testmsg_buffer.front();
            test_msg_pub.publish(msg);
            testmsg_buffer.pop();
            ++last_sent_test_msg_id;
            cout << "[sendTestMsg] buffer send msg: " << msg.data << endl;
        }
        saveLastSentId();
        cout << "[sendTestMsg] publish. buffer size2:" << testmsg_buffer.size() << endl;
    }
    removeHistoryTestMsgFromDisk();
}

void pingClient(const ros::TimerEvent&, const string& ip_to_query) 
{
    std::cout << "pingClient" << std::endl;
    try
    {
        tcp::socket socket(io_service);
        tcp::resolver resolver(io_service);
        tcp::resolver::query query(tcp::v4(), ip_to_query, "11311"); // port with ROS_MASTER
        tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
        boost::asio::connect(socket, endpoint_iterator);
        is_connected = true;
        last_connected_time = std::chrono::steady_clock::now();
        socket.close();
        // cout << "Connected with " << ip_to_query << endl;
    }
    catch (...)
    {
        is_connected = false;
        cerr << "!!Disonnected with " << ip_to_query << endl;
    }

    // check timeout
    if (!is_connected) 
    {
        auto current_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_connected_time).count();
        if (duration > timeout_limit) {
            cerr << "TIME OUT! time: " << duration << "s" << endl;
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "message_buffer_server");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");

    string ip_to_query = "192.168.1.11";
    string test_in_tp;

    ros::param::get("/is_respawn", is_respawn);
    cout << "is_respawn: " << is_respawn << endl;
    priv_nh.getParam("ip_to_query_", ip_to_query);
    cout << "ip_to_query: " << ip_to_query << endl;
    priv_nh.getParam("test_in_tp_", test_in_tp);
    priv_nh.getParam("test_msg_backup_filename_", test_msg_backup_filename);
    cout << "test_msg_backup_filename: " << test_msg_backup_filename << endl;
    priv_nh.getParam("last_sent_filename_", last_sent_filename);
    cout << "last_sent_filename: " << last_sent_filename << endl;

    if(is_respawn)
    {
        cout << "recover" << endl;
        loadLastSentId();
        recoverTestMsgBuffer();
    }
    else
    {
        ofstream outfile(test_msg_backup_filename, ios::trunc | ios::binary);
        ros::param::set("/is_respawn", true);
        is_respawn = true;
    }

    // 初始化 last_connected_time
    last_connected_time = std::chrono::steady_clock::now();

    ros::Subscriber test_msg_sub = nh.subscribe(test_in_tp, 100, testmsgCallback);

    // 创建定时器，每 2 秒执行一次 pingClient 回调函数，传递 ip_to_query 参数
    ros::Timer ping_timer = nh.createTimer(ros::Duration(0.5), boost::bind(pingClient, _1, ip_to_query));
    ros::Timer test_msg_timer = nh.createTimer(ros::Duration(0.5), sendTestMsg);

    // publisher
    test_msg_pub = priv_nh.advertise<std_msgs::String>("test_msg_out", 100);

    // 使用 AsyncSpinner
    ros::AsyncSpinner spinner(4); // 使用 4 个线程
    spinner.start();

    // 主线程继续执行其他任务
    ros::waitForShutdown();

    return 0;
}