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
    std::cout << "called from GodotRosNode::Startup\n";
    rclcpp::init(0, nullptr);
}

void GodotRosNode::Shutdown()
{
    std::cout << "called from GodotRosNode::Shutdown\n";
    rclcpp::shutdown();
}

GodotRosNode::~GodotRosNode()
{
    std::cout << "called from GodotRosNode::~GodotRosNode\n";
}

// GodotRosNode::gett_string_publisher
// GodotRosNode::get_int64_publisher

void GodotRosNode::_bind_methods()
{
    ClassDB::bind_static_method(StringName("GodotRosNode"), D_METHOD("Startup"), &GodotRosNode::Startup);
    ClassDB::bind_static_method(StringName("GodotRosNode"), D_METHOD("Shutdown"), &GodotRosNode::Shutdown);
    // ClassDB::bind_method(D_METHOD("shutdown"), &GodotRosNode::shutdown);

    // ClassDB::bind_method(D_METHOD("add", "value"), &GodotRos::add);
	// ClassDB::bind_method(D_METHOD("reset"), &Summator::reset);
	// ClassDB::bind_method(D_METHOD("shutdown"), &Summator::get_total);
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