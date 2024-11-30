#ifndef GODOT_ROS_SUBSCRIBER_H
#define GODOT_ROS_SUBSCRIBER_H

#include "godot_ros_node.h"

#include <godot_cpp/classes/ref.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace godot
{
class GodotRosSubscriber : public RefCounted
{

// Godot stuff
    GDCLASS(GodotRosSubscriber, RefCounted);
protected:
    static void _bind_methods();

public:
    // Shouldn't need a constructor / deconstructor
    GodotRosSubscriber();
    ~GodotRosSubscriber();

    void init(const Ref<GodotRosNode>& node, const String& topic_name); // TODO: figure out how to take a callback

private:
    using RosType = std_msgs::msg::String;
    using GodotType = godot::String;

    rclcpp::Subscription<RosType>::SharedPtr m_sub;
};

}


#endif