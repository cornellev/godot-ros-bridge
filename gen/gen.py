# NOTE: this is experimental code generation for ros messages into 
# sudo find /opt/ros/humble -type f -name "*.msg"
ros_distro = "humble"
ros_dir = f"/opt/ros/{ros_distro}"

ros_msg_paths = [
    "/opt/ros/humble/share/builtin_interfaces/msg/Time.msg",
    "/opt/ros/humble/share/std_msgs/msg/Header.msg",
    "/opt/ros/humble/share/sensor_msgs/msg/Imu.msg" ]

def parse_ros_msg_file(ros_msg_path):
    ros_msg_fields = []
    with open(ros_msg_path, 'r') as ros_msg_file:
        for line in ros_msg_file:
            line = line.strip()
            if len(line)==0 or line.startswith('#'): continue
            print("line\"",line,"\"")
            ros_msg_type, name = line.strip().split(' ')[:2]
            ros_msg_fields.append((ros_msg_type, name))

    return ros_msg_fields

def is_builtin_type(ros_msg_type):
    file = open("gen/ref_counted.template")
    template = file.read()
    template.replace("$1", ros_msg_type)
    pass

def generate_ros_type():
    pass


for ros_msg_path in ros_msg_paths:
    print(parse_ros_msg_file(ros_msg_path))