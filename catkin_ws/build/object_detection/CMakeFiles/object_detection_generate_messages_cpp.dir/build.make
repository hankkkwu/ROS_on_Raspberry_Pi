# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/think/ros_project/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/think/ros_project/catkin_ws/build

# Utility rule file for object_detection_generate_messages_cpp.

# Include the progress variables for this target.
include object_detection/CMakeFiles/object_detection_generate_messages_cpp.dir/progress.make

object_detection/CMakeFiles/object_detection_generate_messages_cpp: /home/think/ros_project/catkin_ws/devel/include/object_detection/BoundingBox.h
object_detection/CMakeFiles/object_detection_generate_messages_cpp: /home/think/ros_project/catkin_ws/devel/include/object_detection/BoundingBoxes.h


/home/think/ros_project/catkin_ws/devel/include/object_detection/BoundingBox.h: /opt/ros/noetic/lib/gencpp/gen_cpp.py
/home/think/ros_project/catkin_ws/devel/include/object_detection/BoundingBox.h: /home/think/ros_project/catkin_ws/src/object_detection/msg/BoundingBox.msg
/home/think/ros_project/catkin_ws/devel/include/object_detection/BoundingBox.h: /opt/ros/noetic/share/gencpp/msg.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/think/ros_project/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating C++ code from object_detection/BoundingBox.msg"
	cd /home/think/ros_project/catkin_ws/src/object_detection && /home/think/ros_project/catkin_ws/build/catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/think/ros_project/catkin_ws/src/object_detection/msg/BoundingBox.msg -Iobject_detection:/home/think/ros_project/catkin_ws/src/object_detection/msg -Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg -Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg -p object_detection -o /home/think/ros_project/catkin_ws/devel/include/object_detection -e /opt/ros/noetic/share/gencpp/cmake/..

/home/think/ros_project/catkin_ws/devel/include/object_detection/BoundingBoxes.h: /opt/ros/noetic/lib/gencpp/gen_cpp.py
/home/think/ros_project/catkin_ws/devel/include/object_detection/BoundingBoxes.h: /home/think/ros_project/catkin_ws/src/object_detection/msg/BoundingBoxes.msg
/home/think/ros_project/catkin_ws/devel/include/object_detection/BoundingBoxes.h: /home/think/ros_project/catkin_ws/src/object_detection/msg/BoundingBox.msg
/home/think/ros_project/catkin_ws/devel/include/object_detection/BoundingBoxes.h: /opt/ros/noetic/share/gencpp/msg.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/think/ros_project/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating C++ code from object_detection/BoundingBoxes.msg"
	cd /home/think/ros_project/catkin_ws/src/object_detection && /home/think/ros_project/catkin_ws/build/catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/think/ros_project/catkin_ws/src/object_detection/msg/BoundingBoxes.msg -Iobject_detection:/home/think/ros_project/catkin_ws/src/object_detection/msg -Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg -Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg -p object_detection -o /home/think/ros_project/catkin_ws/devel/include/object_detection -e /opt/ros/noetic/share/gencpp/cmake/..

object_detection_generate_messages_cpp: object_detection/CMakeFiles/object_detection_generate_messages_cpp
object_detection_generate_messages_cpp: /home/think/ros_project/catkin_ws/devel/include/object_detection/BoundingBox.h
object_detection_generate_messages_cpp: /home/think/ros_project/catkin_ws/devel/include/object_detection/BoundingBoxes.h
object_detection_generate_messages_cpp: object_detection/CMakeFiles/object_detection_generate_messages_cpp.dir/build.make

.PHONY : object_detection_generate_messages_cpp

# Rule to build all files generated by this target.
object_detection/CMakeFiles/object_detection_generate_messages_cpp.dir/build: object_detection_generate_messages_cpp

.PHONY : object_detection/CMakeFiles/object_detection_generate_messages_cpp.dir/build

object_detection/CMakeFiles/object_detection_generate_messages_cpp.dir/clean:
	cd /home/think/ros_project/catkin_ws/build/object_detection && $(CMAKE_COMMAND) -P CMakeFiles/object_detection_generate_messages_cpp.dir/cmake_clean.cmake
.PHONY : object_detection/CMakeFiles/object_detection_generate_messages_cpp.dir/clean

object_detection/CMakeFiles/object_detection_generate_messages_cpp.dir/depend:
	cd /home/think/ros_project/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/think/ros_project/catkin_ws/src /home/think/ros_project/catkin_ws/src/object_detection /home/think/ros_project/catkin_ws/build /home/think/ros_project/catkin_ws/build/object_detection /home/think/ros_project/catkin_ws/build/object_detection/CMakeFiles/object_detection_generate_messages_cpp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : object_detection/CMakeFiles/object_detection_generate_messages_cpp.dir/depend

