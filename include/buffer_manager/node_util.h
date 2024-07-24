#include <buffer_manager/buffer_manager.h>
#include <unordered_map>

// 创建类型别名，方便管理工厂
using BufferManagerPtr = std::unique_ptr<void, std::function<void(void*)>>;

std::unique_ptr<void, std::function<void(void*)>> createBufferManager(const std::string& type, ros::NodeHandle& nh) {
    static std::unordered_map<std::string, std::function<BufferManagerPtr(ros::NodeHandle&)>> factoryMap = {
        {"submap", [](ros::NodeHandle& nh) -> BufferManagerPtr { return BufferManagerPtr(new BufferManager<dislam_msgs::SubMap>(nh), [](void* ptr){ delete static_cast<BufferManager<dislam_msgs::SubMap>*>(ptr); }); }},
        {"colormap", [](ros::NodeHandle& nh) -> BufferManagerPtr { return BufferManagerPtr(new BufferManager<sensor_msgs::PointCloud2>(nh), [](void* ptr){ delete static_cast<BufferManager<sensor_msgs::PointCloud2>*>(ptr); }); }},
        {"image", [](ros::NodeHandle& nh) -> BufferManagerPtr { return BufferManagerPtr(new BufferManager<sensor_msgs::Image>(nh), [](void* ptr){ delete static_cast<BufferManager<sensor_msgs::Image>*>(ptr); }); }}
    };

    auto it = factoryMap.find(type);
    if (it != factoryMap.end()) {
        return it->second(nh); // 调用相应的创建函数
    }
    return nullptr;
}