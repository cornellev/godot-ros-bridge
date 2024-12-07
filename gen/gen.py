# NOTE: this is experimental code generation for ros messages into 
# sudo find /opt/ros/humble -type f -name "*.msg"
from dataclasses import dataclass
import os

from collections import defaultdict, deque

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

    full_type: str = ""

class CommonTypeConverter:
    def __init__(self, fn=lambda n: n):
        self.fn = fn

    def convert_ros_msg(self, variables: list[str], ros_msg: RosMsg, before, original_msg):
        if convert_ros_msg_to_godot_type(original_msg) in cpp_types:
            return ".".join(['msg']+variables) + " = godot_data;\n"

        if before:
            msg_part = '.'.join(['msg'] + variables)
            godot_part = self.fn('->'.join(['godot_data'] + variables[:-1]) + '.' + variables[-1])

            return f'{msg_part} = {godot_part};\n'
        else:
            msg_part = '.'.join(['msg'] + variables)
            godot_part = self.fn('->'.join(['godot_data'] + variables))

            return f'{msg_part} = {godot_part};\n'

class ArrayTypeConverter:
    def __init__(self):
        super().__init__()
        self.converter = CommonTypeConverter()
    
    def convert_ros_msg(self, variables, ros_msg, outer_msg, original_msg):
        vector_name = "".join(variables)
        godot_part = "->".join(['godot_data'] + variables)
        g = f"std::vector<{convert_ros_msg_to_godot_type(ros_msg)}> {vector_name};\n"
        g += f"""    for (int i = 0; i < {godot_part}.size(); i++) {{
        {vector_name}.push_back({godot_part}[i]);
    }}
"""

        return g


cpp_types = set(["float", "int", "double"])

# TYPES THAT DO NOT NEED TO BE ADDED TO GODOT!
common_types = {
    # ros message type -> godot::type (available in BOTH c++ and godot)
    "Vector3": "godot::Vector3",
    "Quaternion": "godot::Quaternion",
    "Float64": "float",
    "int32": "int",
    "int64": "int",
    "uint32": "int",
    "string": "godot::String",
    "float64": "double",
    "float32": "float"
}

# TODO: the above and below dictionaries could be combined into one if you call
# the generated helper functions for the common types later on and also saves code
# space but I have lost what this code is doing

# TYPES THAT HAVE TRIVIAL CONVERSION FUNCTIONS
common_type_converters = {
    "string": CommonTypeConverter(lambda n: n+".utf8().get_data()"),
    "int32": CommonTypeConverter(),
    "int64": CommonTypeConverter(),
    "uint32": CommonTypeConverter(),
    "float64": CommonTypeConverter(),
    "float32": CommonTypeConverter(),
}


def get_ros_msg_converter(ros_msg_type, is_array):
    if is_array:
        return ArrayTypeConverter()

    if ros_msg_type in common_type_converters:
        return common_type_converters[ros_msg_type]

def convert_ros_msg_field_to_godot_type(ros_msg_field):
    if ros_msg_field.is_array: 
        return "Array"

    if ros_msg_field.type in common_types:
        return common_types[ros_msg_field.type]
    
    return f"Ref<{ros_msg_field.type}>"

def convert_ros_msg_to_godot_type(ros_msg:RosMsg):
    if ros_msg.name in common_types:
        return common_types[ros_msg.name]
    
    return f"Ref<{ros_msg.name}>"

def parse_ros_msg_file(ros_msg_path: str) -> RosMsg:
    ros_msg_fields = []
    with open(ros_msg_path, 'r') as ros_msg_file:
        for line in ros_msg_file:
            line = line.strip()
            if len(line)==0 or line.startswith('#'): continue

            ros_msg_type_full, name = line.strip().split(' ')[:2]
            type_parts= ros_msg_type_full.split("[")

            ros_msg_field = RosMsgField(
                type=os.path.basename(type_parts[0]), 
                name=name, 
                is_array=len(type_parts)>1,
                _full_type=type_parts[0])
            ros_msg_fields.append(ros_msg_field)
    
    msg_name = os.path.basename(os.path.splitext(ros_msg_path)[0])
    full_type = os.path.splitext(ros_msg_path)[0].split("share/")[1].replace("/","::")
    return RosMsg(name=msg_name, fields=ros_msg_fields, full_type=full_type)

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


# TODO: (this is a big one), remove rosmsgfield and make rosmsg point to other rosmsgs
def generate_godot_type_to_ros_conversion(ros_msg_map: dict[str, RosMsg], ros_msg:RosMsg):
    variables = ["msg"]
    ORIGINAL_MESSAGE = ros_msg
    def _impl(ros_msg:RosMsg, variables: list[str], before):
        
        if ros_msg.name in common_type_converters:
            return "    " + get_ros_msg_converter(ros_msg.name, is_array=ros_msg.fields).convert_ros_msg(variables[1:], ros_msg, before.name in common_types, ORIGINAL_MESSAGE)
            # return (f"    {'.'.join(variables)} = {get_ros_msg_converter(ros_msg.name, ros_msg.fields).convert_ros_msg(['godot_data']+variables[1:], ros_msg)};\n")

        g = ""
        for field in ros_msg.fields:
            if field.type in common_type_converters:
                g += _impl(RosMsg(name=field.type, fields=field.is_array), variables + [ field.name], ros_msg)
            else:
                g += _impl(ros_msg_map[field.type], variables + [ field.name ], ros_msg)
        return g
    
    s = ""
    s+=f"{ros_msg.full_type} convert_{ros_msg.name}_to_ros_msg(const {convert_ros_msg_to_godot_type(ros_msg)}& godot_data) {{\n"
    s+=f"    auto msg = {ros_msg.full_type}();\n\n"
    s+=_impl(ros_msg, variables, None)
    s+=("\n    return msg;\n")
    s+=("}\n")
    return s


def generate_ros_publisher_code():
    pass

def generate_ros_subscriber_code():
    pass

def get_package_ordering(ros_msgs: list[RosMsg]):
    def topological_sort(ros_msgs: list[RosMsg]):
        # Build graph and compute indegrees
        graph = defaultdict(set)
        indegree = defaultdict(int)
        
        for ros_msg in ros_msgs:
            module = ros_msg.name
            dependencies = set(field.type for field in ros_msg.fields)
            for dependency in dependencies:
                graph[dependency].add(module)
                indegree[module] += 1
            if module not in indegree:  # Ensure the module is in the indegree map
                indegree[module] = 0
        
        # Initialize queue with nodes having zero indegree
        queue = deque([node for node in indegree if indegree[node] == 0])
        order = []
        
        while queue:
            current = queue.popleft()
            order.append(current)
            
            for neighbor in graph[current]:
                indegree[neighbor] -= 1
                if indegree[neighbor] == 0:
                    queue.append(neighbor)
        
        # Check for cycles
        if len(order) != len(indegree):
            raise ValueError(f"Cycle detected in module dependencies (likely missing dependencies): {set(key for key, degree in indegree.items() if degree>0)}")
        
        return order
    
    order = topological_sort(ros_msgs)
    return order
    # NOTE: types that should not be build into their own ros messages (have godot equivalents)
    # return [ ros_msg_map[ros_msg_name] for ros_msg_name in order ]

def get_ros_msgs_from_paths(ros_msg_paths):
    msgs = [parse_ros_msg_file(ros_msg_path) for ros_msg_path in ros_msg_paths]
    # msgs = [ros_msg for ros_msg in [parse_ros_msg_file(ros_msg_path) for ros_msg_path in ros_msg_paths] if ros_msg.name not in common_types]

    # built_in_types = [ RosMsg(name=name, fields=[]) for name in common_types ]

    return msgs #+ built_in_types


ros_msg_paths = [
    "/opt/ros/humble/share/builtin_interfaces/msg/Time.msg",
    "/opt/ros/humble/share/std_msgs/msg/Header.msg",
    "/opt/ros/humble/share/sensor_msgs/msg/Imu.msg",
    "/opt/ros/humble/share/sensor_msgs/msg/LaserScan.msg",
    "/opt/ros/humble/share/geometry_msgs/msg/Vector3.msg",
    "/opt/ros/humble/share/geometry_msgs/msg/Quaternion.msg",
    "/opt/ros/humble/share/std_msgs/msg/Float64.msg", 
]

ros_msgs = get_ros_msgs_from_paths(ros_msg_paths)
builtin_types = [ RosMsg(name=name, fields=[]) for name in common_types ]
# for ros_msg in ros_msgs:
#     print(ros_msg.name)
# for ros_msg in builtin_types:
#     print(ros_msg.name)

ros_msg_map = {ros_msg.name: ros_msg for ros_msg in ros_msgs}


order = [ ros_msg_map[name] for name in get_package_ordering(ros_msgs + builtin_types) if name not in common_type_converters]
# print('Order:',[ros_msg.name for ros_msg in order])

g = []

for ros_msg in order:
    if ros_msg.name not in common_types:
        g.append(generate_ros_msg_code(ros_msg))

for ros_msg in order:
    if ros_msg.name not in common_type_converters:
        g.append(generate_godot_type_to_ros_conversion(ros_msg_map, ros_msg))

print(open("gen/complete.template").read().replace("$1", "\n".join(g)))
