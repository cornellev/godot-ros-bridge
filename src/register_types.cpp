#include "register_types.h"

#include "godot_ros_types.h"
#include "godot_ros_node.h"
#include "godot_ros_publisher.h"
#include "godot_ros_subscriber.h"

#include <gdextension_interface.h>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/godot.hpp>

using namespace godot;

void initialize_example_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}

	// Ros Messages
	GDREGISTER_CLASS(LaserScan);
	GDREGISTER_CLASS(Imu);

	// Ros Publishers
	GDREGISTER_CLASS(GodotRosNode);
	GDREGISTER_CLASS(GodotRosStringPublisher);
	GDREGISTER_CLASS(GodotRosFloat32Publisher);
	GDREGISTER_CLASS(GodotRosFloat64Publisher);
	GDREGISTER_CLASS(GodotRosFloat64ArrayPublisher);
	GDREGISTER_CLASS(GodotRosLaserScanPublisher);
	GDREGISTER_CLASS(GodotRosImuPublisher);
	GDREGISTER_CLASS(GodotRosImagePublisher);

	// Ros Subscribers
	GDREGISTER_CLASS(GodotRosStringSubscriber);
	GDREGISTER_CLASS(GodotRosInt32Subscriber);
	GDREGISTER_CLASS(GodotRosFloat32Subscriber);
	GDREGISTER_CLASS(GodotRosFloat64Subscriber);
	GDREGISTER_CLASS(GodotRosFloat32ArraySubscriber);

}

void uninitialize_example_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}
}

extern "C" {
// Initialization.
GDExtensionBool GDE_EXPORT example_library_init(GDExtensionInterfaceGetProcAddress p_get_proc_address, const GDExtensionClassLibraryPtr p_library, GDExtensionInitialization *r_initialization) {
	godot::GDExtensionBinding::InitObject init_obj(p_get_proc_address, p_library, r_initialization);

	init_obj.register_initializer(initialize_example_module);
	init_obj.register_terminator(uninitialize_example_module);
	init_obj.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_SCENE);

	return init_obj.init();
}
}