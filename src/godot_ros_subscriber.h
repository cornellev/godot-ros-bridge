#ifndef GODOT_ROS_SUBSCRIBER_H
#define GODOT_ROS_SUBSCRIBER_H

#include "godot_ros_node.h"

#include <godot_cpp/classes/ref.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>

namespace godot
{
class GodotRosBoolSubscriber : public RefCounted {
    GDCLASS(GodotRosBoolSubscriber, RefCounted);

protected:
    static void _bind_methods() {
        ClassDB::bind_method(D_METHOD("init", "node", "topic"), &GodotRosBoolSubscriber::init);
        ADD_SIGNAL(MethodInfo("message_received", PropertyInfo(Variant::BOOL, "data")));
    }

public:
    void init(const Ref<GodotRosNode>& node, const String& topic_name) {
        m_sub = node->m_node->create_subscription<RosType>(
            topic_name.utf8().get_data(),
            10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                // Emit the signal with the received boolean value
                emit_signal("message_received", msg->data);
            }
        );
    }

private:
    using RosType = std_msgs::msg::Bool;
    using GodotType = bool; // The corresponding Godot type for booleans
    rclcpp::Subscription<RosType>::SharedPtr m_sub;
};


class GodotRosStringSubscriber : public RefCounted
{

// Godot stuff
    GDCLASS(GodotRosStringSubscriber, RefCounted);
protected:
    static void _bind_methods()
    {
        ClassDB::bind_method(D_METHOD("init", "node", "topic"), &GodotRosStringSubscriber::init);

        ADD_SIGNAL(MethodInfo("message_received", PropertyInfo(Variant::STRING, "data")));
    }

public:

    void init(const Ref<GodotRosNode>& node, const String& topic_name)
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

private:
    using RosType = std_msgs::msg::String;
    using GodotType = godot::String;

    rclcpp::Subscription<RosType>::SharedPtr m_sub;
};


class GodotRosInt32Subscriber : public RefCounted
{
    GDCLASS(GodotRosInt32Subscriber, RefCounted);
protected:
    static void _bind_methods()
    {
        ClassDB::bind_method(D_METHOD("init", "node", "topic"), &GodotRosInt32Subscriber::init);
        ADD_SIGNAL(MethodInfo("message_received", PropertyInfo(Variant::INT, "data")));
    }

private:
    using RosType = std_msgs::msg::Int32;
    using GodotType = int32_t;

    rclcpp::Subscription<RosType>::SharedPtr m_sub;

public:

    void init(const Ref<GodotRosNode>& node, const String& topic_name)
    {
        m_sub = node->m_node->create_subscription<RosType>(
            topic_name.utf8().get_data(), 
            10, 
            // use shared_ptr<RosType> instead
            [this](const RosType::SharedPtr msg) {
                // Emit the signal when data is received
                emit_signal("message_received", GodotType(msg->data));
            });
    }
};

class GodotRosFloat64Subscriber : public RefCounted
{
    GDCLASS(GodotRosFloat64Subscriber, RefCounted);
protected:
    static void _bind_methods()
    {
        ClassDB::bind_method(D_METHOD("init", "node", "topic"), &GodotRosFloat64Subscriber::init);
        ADD_SIGNAL(MethodInfo("message_received", PropertyInfo(Variant::FLOAT, "data")));
    }

private:
    using RosType = std_msgs::msg::Float64;
    using GodotType = double;

    rclcpp::Subscription<RosType>::SharedPtr m_sub;

public:

    void init(const Ref<GodotRosNode>& node, const String& topic_name)
    {
        m_sub = node->m_node->create_subscription<RosType>(
            topic_name.utf8().get_data(), 
            10, 
            // use shared_ptr<RosType> instead
            [this](const RosType::SharedPtr msg) {
                // Emit the signal when data is received
                emit_signal("message_received", GodotType(msg->data));
            });
    }
};

class GodotRosFloat32Subscriber : public RefCounted
{
    GDCLASS(GodotRosFloat32Subscriber, RefCounted);
protected:
    static void _bind_methods()
    {
        ClassDB::bind_method(D_METHOD("init", "node", "topic"), &GodotRosFloat32Subscriber::init);
        ADD_SIGNAL(MethodInfo("message_received", PropertyInfo(Variant::FLOAT, "data")));
    }

private:
    using RosType = std_msgs::msg::Float32;
    using GodotType = float;

    rclcpp::Subscription<RosType>::SharedPtr m_sub;

public:

    void init(const Ref<GodotRosNode>& node, const String& topic_name)
    {
        m_sub = node->m_node->create_subscription<RosType>(
            topic_name.utf8().get_data(), 
            10, 
            // use shared_ptr<RosType> instead
            [this](const RosType::SharedPtr msg) {
                // Emit the signal when data is received
                emit_signal("message_received", GodotType(msg->data));
            });
    }
};

class GodotRosFloat32ArraySubscriber : public RefCounted {
    GDCLASS(GodotRosFloat32ArraySubscriber, RefCounted);

protected:
    static void _bind_methods() {
        ClassDB::bind_method(D_METHOD("init", "node", "topic"), &GodotRosFloat32ArraySubscriber::init);
        ADD_SIGNAL(MethodInfo("message_received", PropertyInfo(Variant::PACKED_FLOAT32_ARRAY, "data")));
    }

private:
    using RosType = std_msgs::msg::Float32MultiArray;
    using GodotType = PackedFloat32Array;

    rclcpp::Subscription<RosType>::SharedPtr m_sub;

public:
    void init(const Ref<GodotRosNode>& node, const String& topic_name) 
    {
        // Create the subscription to the ROS topic
        m_sub = node->m_node->create_subscription<RosType>(
            topic_name.utf8().get_data(),
            10, // Queue size
            [this](const RosType::SharedPtr msg) {
                // Convert the ROS message data (std::vector<float>) to a Godot PackedFloat32Array
                GodotType godot_array;
                godot_array.resize(msg->data.size());
                for (size_t i = 0; i < msg->data.size(); ++i) {
                    godot_array[i] = msg->data[i];
                }
                // Emit the signal with the array
                emit_signal("message_received", godot_array);
            });
    }
};

}


#endif