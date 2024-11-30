#include "./godot_ros_publisher.h"

#include <iostream>

using namespace godot;


void GodotRosPublisher::_bind_methods()
{
    ClassDB::bind_method(D_METHOD("init", "node", "topic_name", "qos"), &GodotRosPublisher::init);
    ClassDB::bind_method(D_METHOD("publish"), &GodotRosPublisher::publish);
}

GodotRosPublisher::GodotRosPublisher()
{
    // RosType data{topic_name.utf8().get_data()};
    std::cout << "Creating ros publisher\n";
}

void GodotRosPublisher::init(const Ref<GodotRosNode>& node, const godot::String& topic_name, uint64_t qos)
{
    std::cout << "Initializing ros publisher\n";
    m_pub = node->m_node->create_publisher<RosType>(topic_name.utf8().get_data(), qos);
}

GodotRosPublisher::~GodotRosPublisher()
{
    std::cout << "Destroying ros publisher\n";
}

void GodotRosPublisher::publish(const godot::String& data)
{
    std::cout << "Publishing on ros publisher\n";
    RosType msg = RosType();
    msg.data = data.utf8().get_data();
    m_pub->publish(msg);
}