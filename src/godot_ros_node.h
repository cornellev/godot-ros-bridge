#ifndef GODOT_ROS_NODE_HPP
#define GODOT_ROS_NODE_HPP

#include <godot_cpp/classes/ref.hpp>

#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace godot {
class GodotRosNode : public RefCounted {

// Godot Stuff
GDCLASS(GodotRosNode, RefCounted)
protected:
    static void _bind_methods();

// Constructors
public:
    GodotRosNode();
    ~GodotRosNode();

    // static void startup();
    static void Shutdown();
    static void Startup();

private:
    std::shared_ptr<rclcpp::Node> m_node;

};
}

#endif
