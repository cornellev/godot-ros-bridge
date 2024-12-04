#include "godot_ros_node.h"

#include <iostream>

using namespace godot;

GodotRosNode::GodotRosNode()
    : m_node{std::make_shared<rclcpp::Node>("godot_ros_node")}
{
    // init()

    // rclcpp::init(0, nullptr);
    // m_node = std::make_shared<rclcpp::Node>("godot_ros_node");
}

void GodotRosNode::Startup()
{
    rclcpp::init(0, nullptr);
}

void GodotRosNode::Shutdown()
{
    rclcpp::shutdown();
}

GodotRosNode::~GodotRosNode()
{
}

// GodotRosNode::gett_string_publisher
// GodotRosNode::get_int64_publisher

void GodotRosNode::_bind_methods()
{
    ClassDB::bind_static_method(StringName("GodotRosNode"), D_METHOD("Startup"), &GodotRosNode::Startup);
    ClassDB::bind_static_method(StringName("GodotRosNode"), D_METHOD("Shutdown"), &GodotRosNode::Shutdown);
    // ClassDB::bind_method(D_METHOD("shutdown"), &GodotRosNode::shutdown);

    ClassDB::bind_method(D_METHOD("spin_some"), &GodotRosNode::spin_some);
    // ClassDB::bind_method(D_METHOD("add", "value"), &GodotRos::add);
	// ClassDB::bind_method(D_METHOD("reset"), &Summator::reset);
	// ClassDB::bind_method(D_METHOD("shutdown"), &Summator::get_total);
}

void GodotRosNode::spin_some()
{
    rclcpp::spin_some(m_node);
}

// void GodotRosNode::startup()
// {
//     rclcpp::init(0, nullptr);
// }

// void GodotRosNode::shutdown()
// {
//     rclcpp::shutdown();
// }



// GodotRosPublisher GodotRosNode::get_publisher(const godot::String& )