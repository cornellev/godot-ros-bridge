import os
from pathlib import Path

EnsureSConsVersion(4, 0)


try:
    Import("env")
except Exception:
    # Default tools with no platform defaults to gnu toolchain.
    # We apply platform specific toolchains via our custom tools.
    env = Environment(tools=["default"], PLATFORM="")

env.PrependENVPath("PATH", os.getenv("PATH"))

# Paths to Godot-CPP precompiled library and include files
godot_cpp_dir = "godot-cpp"
godot_cpp_include = os.path.join(godot_cpp_dir, "include")
godot_cpp_bindings_include = os.path.join(godot_cpp_dir, "gen", "include")
godot_cpp_lib_path = os.path.join(godot_cpp_dir, "bin")
godot_cpp_gdextension_path = os.path.join(godot_cpp_dir, "gdextension")

# Define ROS paths
ros_distro = "humble"
ros_dir = f"/opt/ros/{ros_distro}"

def getSubDirs(base_path: str, with_base_path: bool = True):
    if not base_path.endswith("/"):
        base_path += "/"
    sub_dirs = [name for name in os.listdir(base_path) if os.path.isdir(base_path + name)]
    if with_base_path:
        sub_dirs = [f"{base_path}/{name}" for name in sub_dirs]
    return sub_dirs

def getLibNames(base_path: str):
    if not base_path.endswith("/"):
        base_path += "/"
    lib_dirs = [
        Path(name).stem
        for name in os.listdir(base_path)
        if os.path.isfile(base_path + name) and (name.endswith(".so") or name.endswith(".a"))
    ]
    return lib_dirs

# Get ROS include and library paths
ros_includes = getSubDirs(ros_dir + "/include")
ros_lib_path = ros_dir + "/lib"
ros_libs = getLibNames(ros_lib_path)

# Add include paths and libraries
env.Append(CPPPATH=[godot_cpp_gdextension_path, godot_cpp_include, godot_cpp_bindings_include, "src/"] + ros_includes)
env.Append(LIBPATH=[ros_lib_path, godot_cpp_lib_path])
env.Append(LIBS=["stdc++", "libgodot-cpp.linux.template_debug.arm64.a"] + ros_libs)

# Specify your source files
sources = Glob("src/*.cpp")

# Compile your shared library (only going to compile for linux lol)
env["SHLIBSUFFIX"] = ".so"
env["suffix"] = ".linux.template_debug.arm64"  
library = env.SharedLibrary(
    "bin/libgdexample{}{}".format(env["suffix"], env["SHLIBSUFFIX"]),
    source=sources,
)

Default(library)
