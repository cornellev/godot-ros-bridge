# godot-ros-bridge

Godot extension that brings a subset of ROS functionality to Godot projects.

## Instructions

### Installation

```
git clone --recurse-submodules -j8 git@github.com:cornellev/godot-ros-bridge.git
```

or 

```
git clone git@github.com:cornellev/godot-ros-bridge.git
cd godot-ros-bridge/
git submodule update --init --recursive
```

### Compilation

Compiling for the first time, must build godot-cpp *first*:
```
cd godot-ros-bridge/godot-cpp/
scons -j8 platform=linux # This step may take 5-10 minutes
cd ../
scons -j8 platform=linux # Builds godot extension to a bin folder
```

All subsequent times, only need to compile godot-ros-bridge itself:
```
cd godot-ros-bridge/
scons -j8 platform=linux # Builds godot extension to a bin folder
```

Then, copy the necessary files into the bin folder of your copy of `cev-godot-sim`:
```
cd godot-ros-bridge/
cp gdexample.gdextension /path/to/cev-godot-sim/bin/
cp bin/libgdexample.<extension> /path/to/cev-godot-sim/bin/
```

## Next steps

Need to cross-compile and add releases for different platforms (and appropiately update SConstruct to handle this).

It may be possible to automatically generate subscribers / publishers (see gen).