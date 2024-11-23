#include "gdexample.h"
#include <godot_cpp/core/class_db.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace godot;

void GDExample::_bind_methods() {
}

GDExample::GDExample() {
	// Initialize any variables here.
	time_passed = 0.0;

	rclcpp::init(0, nullptr);
    
    m_node = std::make_shared<rclcpp::Node>("godot_talker_node");

    rclcpp::QoS qos(rclcpp::KeepLast(7));
    m_pub = m_node->create_publisher<std_msgs::msg::String>("talker", qos);

}

GDExample::~GDExample() {
	rclcpp::shutdown();
}

void GDExample::_process(double delta) {
	time_passed += delta;

	m_msg = std::make_unique<std_msgs::msg::String>();
    m_msg->data = "Hello from Godot: " + std::to_string(delta);
    RCLCPP_INFO(m_node->get_logger(), "Publishing: '%s'", m_msg->data.c_str());

    m_pub->publish(std::move(m_msg));


	Vector2 new_position = Vector2(10.0 + (10.0 * sin(time_passed * 2.0)), 10.0 + (10.0 * cos(time_passed * 1.5)));

	set_position(new_position);
}