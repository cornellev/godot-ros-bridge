#include "godot_ros_subscriber.h"
using namespace godot;

#include <iostream>

void GodotRosStringSubscriber::_bind_methods()
{
    ClassDB::bind_method(D_METHOD("init", "node", "topic"), &GodotRosStringSubscriber::init);

    ADD_SIGNAL(MethodInfo("message_received", PropertyInfo(Variant::STRING, "data")));
}

GodotRosStringSubscriber::GodotRosStringSubscriber()
{
    // std::cout
}

GodotRosStringSubscriber::~GodotRosStringSubscriber()
{
    
}

void GodotRosStringSubscriber::init(const Ref<GodotRosNode>& node, const String& topic_name)
{
    m_sub = node->m_node->create_subscription<RosType>(
        topic_name.utf8().get_data(), 
        10, 
        // use shared_ptr<RosType> instead
        [this](const std_msgs::msg::String::SharedPtr msg) {
            // Emit the signal when data is received
            emit_signal("message_received", String(msg->data.c_str()));
        });
}