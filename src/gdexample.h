#ifndef GDEXAMPLE_H
#define GDEXAMPLE_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <godot_cpp/classes/sprite2d.hpp>

namespace godot {

class GDExample : public Sprite2D {
	GDCLASS(GDExample, Sprite2D)

private:
	double time_passed;

protected:
	static void _bind_methods();

	// replace rclcpp::Node with your custom node
  std::shared_ptr<rclcpp::Node> m_node;

  // publisher
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_pub;

  // message to publish
  std::unique_ptr<std_msgs::msg::String> m_msg;

public:
	GDExample();
	~GDExample();

	void _process(double delta) override;
};

}

#endif