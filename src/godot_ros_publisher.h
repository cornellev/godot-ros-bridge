#ifndef GODOT_ROS_PUBLISHER_H
#define GODOT_ROS_PUBLISHER_H

#include "godot_ros_node.h"

#include <godot_cpp/classes/ref.hpp>
#include <godot_cpp/godot.hpp>
#include <godot_cpp/variant/variant.hpp>


#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace godot
{

// template <typename Publisher>
// void bind_publisher_methods()
// {
//     ClassDB::bind_method(D_METHOD("init", "node", "topic_name", "qos"), &Publisher::init);
//     ClassDB::bind_method(D_METHOD("publish"), &Publisher::publish);
// }

// template <typename RosMsg, typename GodotType>
// class GodotRosPublisher : public RefCounted
// {
// public:

//     virtual void init(const Ref<GodotRosNode>& node, const godot::String& topic_name, uint64_t qos=10)
//     {    
//         std::cout << "Initializing ros publisher\n";
//         m_pub = node->m_node->create_publisher<RosMsg>(topic_name.utf8().get_data(), qos);
//     }

//     virtual void publish(const GodotType& godot_data)
//     {
//         m_pub->publish(godot_data_to_ros_msg(godot_data));
//     }

// private:
//     virtual RosMsg godot_data_to_ros_msg(const GodotType& godot_data) = 0;
//     typename rclcpp::Publisher<RosMsg>::SharedPtr m_pub;

// };

// #define GODOT_ROS_PUBLISHER(TypeName, _RosMsg, _GodotType, _ConversionFunc) \
//     class Godot##

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
        std::cout << "Initializing ros publisher\n";
        m_pub = node->m_node->create_publisher<RosMsg>(topic_name.utf8().get_data(), qos);
    }

    void publish(const GodotType& godot_data)
    {
        m_pub->publish(godot_data_to_ros_msg(godot_data));
    }

    rclcpp::Publisher<RosMsg>::SharedPtr m_pub;
};


struct LaserScan : public RefCounted {
    GDCLASS(LaserScan, RefCounted);
public:
    float angle_min;
    float angle_max;
    float angle_increment;
    float time_increment;
    float scan_time;
    float range_min;
    float range_max;
    std::vector<float> ranges;
    std::vector<float> intensities;

    static void _bind_methods() {
        // Binding getter and setter for each property
        ClassDB::bind_method(D_METHOD("get_angle_min"), &LaserScan::get_angle_min);
        ClassDB::bind_method(D_METHOD("set_angle_min", "value"), &LaserScan::set_angle_min);
        
        ClassDB::bind_method(D_METHOD("get_angle_max"), &LaserScan::get_angle_max);
        ClassDB::bind_method(D_METHOD("set_angle_max", "value"), &LaserScan::set_angle_max);

        ClassDB::bind_method(D_METHOD("get_angle_increment"), &LaserScan::get_angle_increment);
        ClassDB::bind_method(D_METHOD("set_angle_increment", "value"), &LaserScan::set_angle_increment);

        ClassDB::bind_method(D_METHOD("get_time_increment"), &LaserScan::get_time_increment);
        ClassDB::bind_method(D_METHOD("set_time_increment", "value"), &LaserScan::set_time_increment);

        ClassDB::bind_method(D_METHOD("get_scan_time"), &LaserScan::get_scan_time);
        ClassDB::bind_method(D_METHOD("set_scan_time", "value"), &LaserScan::set_scan_time);

        ClassDB::bind_method(D_METHOD("get_range_min"), &LaserScan::get_range_min);
        ClassDB::bind_method(D_METHOD("set_range_min", "value"), &LaserScan::set_range_min);

        ClassDB::bind_method(D_METHOD("get_range_max"), &LaserScan::get_range_max);
        ClassDB::bind_method(D_METHOD("set_range_max", "value"), &LaserScan::set_range_max);

        // Binding the ranges and intensities array getters and setters
        ClassDB::bind_method(D_METHOD("get_ranges"), &LaserScan::get_ranges);
        ClassDB::bind_method(D_METHOD("set_ranges", "values"), &LaserScan::set_ranges);

        ClassDB::bind_method(D_METHOD("get_intensities"), &LaserScan::get_intensities);
        ClassDB::bind_method(D_METHOD("set_intensities", "values"), &LaserScan::set_intensities);

        ClassDB::bind_method(D_METHOD("get_frame_id"), &LaserScan::get_frame_id);
        ClassDB::bind_method(D_METHOD("set_frame_id", "value"), &LaserScan::set_frame_id);
    }

    godot::String get_frame_id() const { return godot::String(frame_id.c_str()); }
    void set_frame_id(const godot::String& value) { frame_id = value.utf8().get_data(); }

    // Getters and Setters for each property
    float get_angle_min() const { return angle_min; }
    void set_angle_min(float value) { angle_min = value; }

    float get_angle_max() const { return angle_max; }
    void set_angle_max(float value) { angle_max = value; }

    float get_angle_increment() const { return angle_increment; }
    void set_angle_increment(float value) { angle_increment = value; }

    float get_time_increment() const { return time_increment; }
    void set_time_increment(float value) { time_increment = value; }

    float get_scan_time() const { return scan_time; }
    void set_scan_time(float value) { scan_time = value; }

    float get_range_min() const { return range_min; }
    void set_range_min(float value) { range_min = value; }

    float get_range_max() const { return range_max; }
    void set_range_max(float value) { range_max = value; }

    // Getter and Setter for the ranges vector
    Array get_ranges() const {
        Array arr;
        for (float range : ranges) {
            arr.append(range);
        }
        return arr;
    }

    void set_ranges(const Array &array) {
        ranges.clear();
        for (int i = 0; i < array.size(); i++) {
            Variant element = array[i];
            if (element.get_type() == Variant::FLOAT) {
                ranges.push_back(element);
            } else {
                // Godot::print_warning("Array contains non-float elements!", "LaserScan", "set_ranges", __FILE__, __LINE__);
                return;
            }
        }
    }

    // Getter and Setter for the intensities vector
    Array get_intensities() const {
        Array arr;
        for (float intensity : intensities) {
            arr.append(intensity);
        }
        return arr;
    }

    void set_intensities(const Array &array) {
        intensities.clear();
        for (int i = 0; i < array.size(); i++) {
            Variant element = array[i];
            if (element.get_type() == Variant::FLOAT) {
                intensities.push_back(element);
            } else {
                // Godot::print_error("Array contains non-float elements!", "LaserScan", "set_intensities", __FILE__, __LINE__);
                return;
            }
        }
    }
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

}

#endif