#ifndef GODOT_ROS_NODE_HPP
#define GODOT_ROS_NODE_HPP

// #include "godot_ros_publisher.h"


#include <godot_cpp/classes/ref.hpp>

#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace godot {
    class GodotRosPublisher;
    class GodotRosSubscriber;

class GodotRosNode : public RefCounted {

// This is dumb I should just make the shared pointer public at this rate
friend class GodotRosPublisher;
friend class GodotRosSubscriber;

// Godot Stuff
GDCLASS(GodotRosNode, RefCounted)
protected:
    static void _bind_methods();

// Static methods
public:
    // Could extract these into another class that extends from Node2D called GlobalRosManager something
    static void Shutdown();
    static void Startup();

// Constructors
public:
    GodotRosNode();
    ~GodotRosNode();

    // TODO: figuring out the right use case requires knowing more 
    // about Godot and ROS threading internals, spin_some will have to do for now
    void spin_some();

private:
    std::shared_ptr<rclcpp::Node> m_node;

};
}

#endif
