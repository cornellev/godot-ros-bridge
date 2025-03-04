#ifndef GODOT_ROS_TYPES_H
#define GODOT_ROS_TYPES_H


#include <godot_cpp/classes/ref.hpp>
#include <godot_cpp/godot.hpp>
#include <godot_cpp/variant/variant.hpp>
#include <godot_cpp/classes/image.hpp>


#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace godot
{

class Imu : public RefCounted
{
    GDCLASS(Imu, RefCounted);
protected:
    static void _bind_methods()
    {
        ClassDB::bind_method(D_METHOD("get_frame_id"), &Imu::get_frame_id);
        ClassDB::bind_method(D_METHOD("set_frame_id", "frame_id"), &Imu::set_frame_id);
    
        ClassDB::bind_method(D_METHOD("get_orientation"), &Imu::get_orientation);
        ClassDB::bind_method(D_METHOD("set_orientation", "orientation"), &Imu::set_orientation);

        ClassDB::bind_method(D_METHOD("get_angular_velocity"), &Imu::get_angular_velocity);
        ClassDB::bind_method(D_METHOD("set_angular_velocity", "angular_velocity"), &Imu::set_angular_velocity);

        ClassDB::bind_method(D_METHOD("get_linear_acceleration"), &Imu::get_angular_velocity);
        ClassDB::bind_method(D_METHOD("set_linear_acceleration", "linear_acceleration"), &Imu::set_linear_acceleration);
    }

public:
    godot::String frame_id;
    godot::Quaternion orientation;
    godot::Vector3 angular_velocity;
    godot::Vector3 linear_acceleration;

    const godot::String &get_frame_id() { return frame_id; }
    void set_frame_id(const godot::String& frame_id) { this->frame_id = frame_id; }

    const godot::Quaternion& get_orientation() { return orientation; }
    void set_orientation(const godot::Quaternion& orientation) { this->orientation = orientation; }

    const godot::Vector3& get_angular_velocity() { return angular_velocity; }
    void set_angular_velocity(const godot::Vector3& angular_velocity) { this->angular_velocity = angular_velocity; }

    const godot::Vector3& get_linear_acceleration() { return linear_acceleration; }
    void set_linear_acceleration(const godot::Vector3& linear_acceleration) { this->linear_acceleration = linear_acceleration; }
};


/*
TODO: make header message type for godot
(handle the built in types: https://wiki.ros.org/msg)

swap std::string frame_id with the afforementioned header type
*/

struct LaserScan : public RefCounted {
    GDCLASS(LaserScan, RefCounted);
public:
    std::string frame_id;
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

}


#endif