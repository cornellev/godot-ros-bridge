#ifndef GODOT_ROS_PUBLISHER_H
#define GODOT_ROS_PUBLISHER_H

#include "godot_ros_node.h"
#include "godot_ros_types.h"

#include <godot_cpp/classes/ref.hpp>
#include <godot_cpp/godot.hpp>
#include <godot_cpp/variant/variant.hpp>
#include <godot_cpp/classes/image.hpp>
#include <godot_cpp/variant/packed_float64_array.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace godot
{

class GodotRosStringPublisher : public RefCounted
{
    using RosMsg = std_msgs::msg::String;
    using GodotType = godot::String;

    GDCLASS(GodotRosStringPublisher, RefCounted);
    
    static void _bind_methods()
    {
        // bind_publisher_methods<GodotRosStringPublisher>();
        ClassDB::bind_method(D_METHOD("init", "node", "topic_name", "qos"), &GodotRosStringPublisher::init);
        ClassDB::bind_method(D_METHOD("publish"), &GodotRosStringPublisher::publish);
    }

private:
    std_msgs::msg::String godot_data_to_ros_msg(const godot::String& godot_string)
    {
        auto msg = std_msgs::msg::String();
        msg.data = godot_string.utf8().get_data();
        return msg;
    }

    void init(const Ref<GodotRosNode>& node, const godot::String& topic_name, uint64_t qos=10)
    {    
        m_pub = node->m_node->create_publisher<RosMsg>(topic_name.utf8().get_data(), qos);
    }

    void publish(const GodotType& godot_data)
    {
        m_pub->publish(godot_data_to_ros_msg(godot_data));
    }

    rclcpp::Publisher<RosMsg>::SharedPtr m_pub;
};

class GodotRosFloat64Publisher : public RefCounted
{
    using RosMsg = std_msgs::msg::Float64;
    using GodotType = double; // screw it

    GDCLASS(GodotRosFloat64Publisher, RefCounted);
    
    static void _bind_methods()
    {
        ClassDB::bind_method(D_METHOD("init", "node", "topic_name", "qos"), &GodotRosFloat64Publisher::init);
        ClassDB::bind_method(D_METHOD("publish"), &GodotRosFloat64Publisher::publish);
    }

private:
    RosMsg godot_data_to_ros_msg(const GodotType& godot_data)
    {
        auto msg = RosMsg();
        msg.data = godot_data;
        return msg;
    }

    void init(const Ref<GodotRosNode>& node, const godot::String& topic_name, uint64_t qos=10)
    {    
        m_pub = node->m_node->create_publisher<RosMsg>(topic_name.utf8().get_data(), qos);
    }

    void publish(const GodotType& godot_data)
    {
        m_pub->publish(godot_data_to_ros_msg(godot_data));
    }

    rclcpp::Publisher<RosMsg>::SharedPtr m_pub;
};

class GodotRosFloat32Publisher : public RefCounted
{
    using RosMsg = std_msgs::msg::Float32;
    using GodotType = float; // screw it

    GDCLASS(GodotRosFloat32Publisher, RefCounted);
    
    static void _bind_methods()
    {
        ClassDB::bind_method(D_METHOD("init", "node", "topic_name", "qos"), &GodotRosFloat32Publisher::init);
        ClassDB::bind_method(D_METHOD("publish"), &GodotRosFloat32Publisher::publish);
    }

private:
    RosMsg godot_data_to_ros_msg(const GodotType& godot_data)
    {
        auto msg = RosMsg();
        msg.data = godot_data;
        return msg;
    }

    void init(const Ref<GodotRosNode>& node, const godot::String& topic_name, uint64_t qos=10)
    {    
        m_pub = node->m_node->create_publisher<RosMsg>(topic_name.utf8().get_data(), qos);
    }

    void publish(const GodotType& godot_data)
    {
        m_pub->publish(godot_data_to_ros_msg(godot_data));
    }

    rclcpp::Publisher<RosMsg>::SharedPtr m_pub;
};

class GodotRosLaserScanPublisher : public RefCounted
{
    using RosMsg = sensor_msgs::msg::LaserScan;
    using GodotType = godot::Ref<LaserScan>;

    GDCLASS(GodotRosLaserScanPublisher, RefCounted);
    
    static void _bind_methods()
    {
        ClassDB::bind_method(D_METHOD("init", "node", "topic_name", "qos"), &GodotRosLaserScanPublisher::init);
        ClassDB::bind_method(D_METHOD("publish"), &GodotRosLaserScanPublisher::publish);
    }

private:
    RosMsg godot_data_to_ros_msg(const GodotType& godot_laser_scan)
    {
        auto msg = RosMsg();
        msg.header.frame_id = godot_laser_scan->frame_id;
        msg.angle_min = godot_laser_scan->angle_min;
        msg.angle_max = godot_laser_scan->angle_max;
        msg.angle_increment = godot_laser_scan->angle_increment;
        msg.time_increment = godot_laser_scan->time_increment;
        msg.scan_time = godot_laser_scan->scan_time;
        msg.range_min = godot_laser_scan->range_min;
        msg.range_max = godot_laser_scan->range_max;
        msg.ranges = godot_laser_scan->ranges;
        msg.intensities = godot_laser_scan->intensities;
                
        return msg;
    }

    void init(const Ref<GodotRosNode>& node, const godot::String& topic_name, uint64_t qos=10)
    {    
        m_pub = node->m_node->create_publisher<RosMsg>(topic_name.utf8().get_data(), qos);
    }

    void publish(const GodotType& godot_data)
    {
        m_pub->publish(godot_data_to_ros_msg(godot_data));
    }

    rclcpp::Publisher<RosMsg>::SharedPtr m_pub;
};

class GodotRosImuPublisher : public RefCounted
{
    using RosMsg = sensor_msgs::msg::Imu;
    using GodotType = Ref<Imu>;

    GDCLASS(GodotRosImuPublisher, RefCounted);

    static void _bind_methods()
    {
        ClassDB::bind_method(D_METHOD("init", "node", "topic_name", "qos"), &GodotRosImuPublisher::init);
        ClassDB::bind_method(D_METHOD("publish"), &GodotRosImuPublisher::publish);
    }

private:
    RosMsg godot_data_to_ros_msg(const GodotType& godot_laser_scan)
    {
        auto msg = RosMsg();

        msg.header.frame_id = godot_laser_scan->frame_id.utf8().get_data();

        msg.orientation = geometry_msgs::msg::Quaternion();
        msg.orientation.x = godot_laser_scan->orientation.x;
        msg.orientation.y = godot_laser_scan->orientation.y;
        msg.orientation.z = godot_laser_scan->orientation.z;
        msg.orientation.w = godot_laser_scan->orientation.w;

        msg.angular_velocity = geometry_msgs::msg::Vector3();
        msg.angular_velocity.x = godot_laser_scan->angular_velocity.x;
        msg.angular_velocity.y = godot_laser_scan->angular_velocity.y;
        msg.angular_velocity.z = godot_laser_scan->angular_velocity.z;

        msg.linear_acceleration = geometry_msgs::msg::Vector3();
        msg.linear_acceleration.x = godot_laser_scan->linear_acceleration.x;
        msg.linear_acceleration.y = godot_laser_scan->linear_acceleration.y;
        msg.linear_acceleration.z = godot_laser_scan->linear_acceleration.z;

        return msg;
    }

    void init(const Ref<GodotRosNode>& node, const godot::String& topic_name, uint64_t qos=10)
    {    
        m_pub = node->m_node->create_publisher<RosMsg>(topic_name.utf8().get_data(), qos);
    }

    void publish(const GodotType& godot_data)
    {
        m_pub->publish(godot_data_to_ros_msg(godot_data));
    }

    rclcpp::Publisher<RosMsg>::SharedPtr m_pub;
};

class GodotRosImagePublisher : public RefCounted {
    GDCLASS(GodotRosImagePublisher, RefCounted);

protected:
    static void _bind_methods() {
        ClassDB::bind_method(D_METHOD("init", "node", "topic_name", "qos"), &GodotRosImagePublisher::init);
        ClassDB::bind_method(D_METHOD("publish", "image", "frame_id"), &GodotRosImagePublisher::publish);
    }

private:
    using RosMsg = sensor_msgs::msg::Image;
    using GodotType = Ref<godot::Image>;

    rclcpp::Publisher<RosMsg>::SharedPtr m_pub;

    RosMsg godot_image_to_ros_msg(const GodotType& godot_image, const String& frame_id) {
        auto msg = RosMsg();

        // Set the frame ID
        msg.header.frame_id = frame_id.utf8().get_data();

        // Fill in ROS Image message fields
        msg.height = godot_image->get_height();
        msg.width = godot_image->get_width();
        msg.encoding = "rgba8";  // Assuming Godot Image uses RGBA8 format
        msg.step = msg.width * 4; // 4 bytes per pixel (RGBA)

        // Copy pixel data
        const uint8_t* godot_data = godot_image->get_data().ptr();
        msg.data.assign(godot_data, godot_data + (msg.height * msg.step));

        return msg;
    }

public:
    void init(const Ref<GodotRosNode>& node, const String& topic_name, uint64_t qos = 10) {
        // Create a publisher for the Image topic
        m_pub = node->m_node->create_publisher<RosMsg>(topic_name.utf8().get_data(), qos);
    }

    void publish(const GodotType& godot_image, const String& frame_id) {
        // Publish the ROS Image message
        m_pub->publish(godot_image_to_ros_msg(godot_image, frame_id));
    }
};

class GodotRosFloat64ArrayPublisher : public RefCounted {
    using RosMsg = std_msgs::msg::Float64MultiArray;
    using GodotType = PackedFloat64Array;

    GDCLASS(GodotRosFloat64ArrayPublisher, RefCounted);

protected:
    static void _bind_methods() {
        // Bind the init and publish methods for use from Godot scripts
        ClassDB::bind_method(D_METHOD("init", "node", "topic_name", "qos"), &GodotRosFloat64ArrayPublisher::init);
        ClassDB::bind_method(D_METHOD("publish"), &GodotRosFloat64ArrayPublisher::publish);
    }

private:
    // Convert a Godot PoolRealArray to a ROS Float64MultiArray message.
    RosMsg godot_data_to_ros_msg(const GodotType &godot_array) {
        auto msg = RosMsg();
        // Loop through the Godot array and push each element into the ROS message's data vector.
        for (int i = 0; i < godot_array.size(); ++i) {
            msg.data.push_back(godot_array[i]);
        }
        return msg;
    }

    // Initialize the publisher with the given ROS node, topic name, and QoS.
    void init(const Ref<GodotRosNode> &node, const String &topic_name, uint64_t qos = 10) {
        m_pub = node->m_node->create_publisher<RosMsg>(topic_name.utf8().get_data(), qos);
    }

    // Publish a PoolRealArray by converting it into a ROS message.
    void publish(const GodotType &godot_array) {
        m_pub->publish(godot_data_to_ros_msg(godot_array));
    }

    // The ROS publisher object.
    rclcpp::Publisher<RosMsg>::SharedPtr m_pub;
};


}

#endif