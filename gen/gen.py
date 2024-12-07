# NOTE: this is experimental code generation for ros messages into 
# sudo find /opt/ros/humble -type f -name "*.msg"
from dataclasses import dataclass
import os

@dataclass
class RosMsgField:
    type: str
    name: str
    is_array: bool

    _full_type: str = ""

@dataclass
class RosMsg:
    name: str
    fields: list[RosMsgField]

class RosMsgToGodotTypeConverter:
    def __init__():
        pass

# need to find ros types that already have a ros equivalent
common_types = {
    # ros message type -> godot::type (available in BOTH c++ and godot)
    "Vector3": "godot::Vector3",
    "Quaternion": "godot::Quaternion",
    "int32": "int",
    "int64": "int",
    "uint32": "int",
    "string": "godot::String"
}

def convert_ros_msg_field_to_godot_type(ros_msg_field):
    if ros_msg_field.is_array: 
        return "Array"

    if ros_msg_field.type in common_types:
        return common_types[ros_msg_field.type]
    
    return f"Ref<{ros_msg_field.type}>"

def parse_ros_msg_file(ros_msg_path: str) -> RosMsg:
    ros_msg_fields = []
    with open(ros_msg_path, 'r') as ros_msg_file:
        for line in ros_msg_file:
            line = line.strip()
            if len(line)==0 or line.startswith('#'): continue
            print("line\"",line,"\"")

            ros_msg_type_full, name = line.strip().split(' ')[:2]
            type_parts= ros_msg_type_full.split("[")

            ros_msg_field = RosMsgField(
                type=os.path.basename(type_parts[0]), 
                name=name, 
                is_array=len(type_parts)>1,
                _full_type=type_parts[0])
            ros_msg_fields.append(ros_msg_field)
    
    msg_name = os.path.basename(os.path.splitext(ros_msg_path)[0])
    return RosMsg(name=msg_name, fields=ros_msg_fields)

def generate_ros_msg_code(ros_msg):
    cpp_fields = []
    cpp_methods = []
    classdb_methods = []

    method_template = open("gen/getter_setter.template").read()
    classdb_template = open("gen/classdb_bind.template").read()

    for field in ros_msg.fields:
        godot_type = convert_ros_msg_field_to_godot_type(field)

        cpp_fields.append(f"    {godot_type} {field.name};")
        cpp_methods.append(method_template.replace("$1", godot_type).replace("$2", field.name))
        classdb_methods.append(classdb_template.replace("$1", field.name).replace("$2", ros_msg.name))

    template = open("gen/ref_counted.template").read()
    template = template.replace("$1", ros_msg.name).replace("$2", "\n".join(classdb_methods)).replace("$3", "\n".join(cpp_fields)).replace("$4", "\n".join(cpp_methods))
    return template

def generate_ros_publisher_code():
    pass

def generate_ros_subscriber_code():
    pass


ros_distro = "humble"
ros_dir = f"/opt/ros/{ros_distro}"

ros_msg_paths = [
    "/opt/ros/humble/share/builtin_interfaces/msg/Time.msg",
    "/opt/ros/humble/share/std_msgs/msg/Header.msg",
    "/opt/ros/humble/share/sensor_msgs/msg/Imu.msg" ]



for ros_msg_path in [ ros_msg_paths[2] ]:
    ros_msg = parse_ros_msg_file(ros_msg_path)
    print(ros_msg)
    print(generate_ros_msg_code(ros_msg))