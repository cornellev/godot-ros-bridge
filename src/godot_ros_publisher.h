#ifndef GODOT_ROS_PUBLISHER_H
#define GODOT_ROS_PUBLISHER_H

#include <rclcpp/rclcpp.hpp>
#include <godot_cpp/classes/ref.hpp>

namespace godot
{

class GodotRosPublisher : public RefCounted
{
    GDCLASS(GodotRosPublisher, RefCounted);

public:
    GodotRosPublisher();

};



}

#endif