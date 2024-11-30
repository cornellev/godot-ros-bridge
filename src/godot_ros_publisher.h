#ifndef GODOT_ROS_PUBLISHER_H
#define GODOT_ROS_PUBLISHER_H

#include "godot_ros_node.h"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
// #include <godot_cpp/string.hpp>
#include <godot_cpp/classes/ref.hpp>


namespace godot
{

// for now can only publish strings
class GodotRosPublisher : public RefCounted
{

// Godot stuff
    GDCLASS(GodotRosPublisher, RefCounted);
protected:
    static void _bind_methods();

protected:
    using RosType = std_msgs::msg::String;
    using GodotType = godot::String;

public:
    GodotRosPublisher(); 
    ~GodotRosPublisher();

    void init(const Ref<GodotRosNode>& node, const GodotType& topic_name, uint64_t qos=10);

    void publish(const GodotType& title);

private:
    rclcpp::Publisher<RosType>::SharedPtr m_pub;


};



}

#endif