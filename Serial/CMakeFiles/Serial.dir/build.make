# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canoncical targets will work.
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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/jcluck/ros_workspace/ycp-robot/Serial

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jcluck/ros_workspace/ycp-robot/Serial

# Include any dependencies generated for this target.
include CMakeFiles/Serial.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Serial.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Serial.dir/flags.make

CMakeFiles/Serial.dir/src/Converter.o: CMakeFiles/Serial.dir/flags.make
CMakeFiles/Serial.dir/src/Converter.o: src/Converter.cpp
CMakeFiles/Serial.dir/src/Converter.o: manifest.xml
CMakeFiles/Serial.dir/src/Converter.o: /opt/ros/electric/ros/tools/rospack/manifest.xml
CMakeFiles/Serial.dir/src/Converter.o: /opt/ros/electric/ros/core/roslib/manifest.xml
CMakeFiles/Serial.dir/src/Converter.o: /opt/ros/electric/stacks/ros_comm/messages/std_msgs/manifest.xml
CMakeFiles/Serial.dir/src/Converter.o: /opt/ros/electric/ros/core/rosbuild/manifest.xml
CMakeFiles/Serial.dir/src/Converter.o: /opt/ros/electric/ros/core/roslang/manifest.xml
CMakeFiles/Serial.dir/src/Converter.o: /opt/ros/electric/stacks/ros_comm/utilities/cpp_common/manifest.xml
CMakeFiles/Serial.dir/src/Converter.o: /opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp_traits/manifest.xml
CMakeFiles/Serial.dir/src/Converter.o: /opt/ros/electric/stacks/ros_comm/utilities/rostime/manifest.xml
CMakeFiles/Serial.dir/src/Converter.o: /opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp_serialization/manifest.xml
CMakeFiles/Serial.dir/src/Converter.o: /opt/ros/electric/stacks/ros_comm/utilities/xmlrpcpp/manifest.xml
CMakeFiles/Serial.dir/src/Converter.o: /opt/ros/electric/stacks/ros_comm/tools/rosconsole/manifest.xml
CMakeFiles/Serial.dir/src/Converter.o: /opt/ros/electric/stacks/ros_comm/messages/rosgraph_msgs/manifest.xml
CMakeFiles/Serial.dir/src/Converter.o: /opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp/manifest.xml
CMakeFiles/Serial.dir/src/Converter.o: /home/jcluck/ros_workspace/ycp-robot/robot_msgs/manifest.xml
CMakeFiles/Serial.dir/src/Converter.o: /opt/ros/electric/stacks/ros_comm/messages/std_msgs/msg_gen/generated
CMakeFiles/Serial.dir/src/Converter.o: /opt/ros/electric/stacks/ros_comm/messages/rosgraph_msgs/msg_gen/generated
CMakeFiles/Serial.dir/src/Converter.o: /opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp/msg_gen/generated
CMakeFiles/Serial.dir/src/Converter.o: /opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp/srv_gen/generated
CMakeFiles/Serial.dir/src/Converter.o: /home/jcluck/ros_workspace/ycp-robot/robot_msgs/msg_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/jcluck/ros_workspace/ycp-robot/Serial/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/Serial.dir/src/Converter.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/Serial.dir/src/Converter.o -c /home/jcluck/ros_workspace/ycp-robot/Serial/src/Converter.cpp

CMakeFiles/Serial.dir/src/Converter.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Serial.dir/src/Converter.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/jcluck/ros_workspace/ycp-robot/Serial/src/Converter.cpp > CMakeFiles/Serial.dir/src/Converter.i

CMakeFiles/Serial.dir/src/Converter.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Serial.dir/src/Converter.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/jcluck/ros_workspace/ycp-robot/Serial/src/Converter.cpp -o CMakeFiles/Serial.dir/src/Converter.s

CMakeFiles/Serial.dir/src/Converter.o.requires:
.PHONY : CMakeFiles/Serial.dir/src/Converter.o.requires

CMakeFiles/Serial.dir/src/Converter.o.provides: CMakeFiles/Serial.dir/src/Converter.o.requires
	$(MAKE) -f CMakeFiles/Serial.dir/build.make CMakeFiles/Serial.dir/src/Converter.o.provides.build
.PHONY : CMakeFiles/Serial.dir/src/Converter.o.provides

CMakeFiles/Serial.dir/src/Converter.o.provides.build: CMakeFiles/Serial.dir/src/Converter.o
.PHONY : CMakeFiles/Serial.dir/src/Converter.o.provides.build

CMakeFiles/Serial.dir/src/DrRobotMotionSensorDriver.o: CMakeFiles/Serial.dir/flags.make
CMakeFiles/Serial.dir/src/DrRobotMotionSensorDriver.o: src/DrRobotMotionSensorDriver.cpp
CMakeFiles/Serial.dir/src/DrRobotMotionSensorDriver.o: manifest.xml
CMakeFiles/Serial.dir/src/DrRobotMotionSensorDriver.o: /opt/ros/electric/ros/tools/rospack/manifest.xml
CMakeFiles/Serial.dir/src/DrRobotMotionSensorDriver.o: /opt/ros/electric/ros/core/roslib/manifest.xml
CMakeFiles/Serial.dir/src/DrRobotMotionSensorDriver.o: /opt/ros/electric/stacks/ros_comm/messages/std_msgs/manifest.xml
CMakeFiles/Serial.dir/src/DrRobotMotionSensorDriver.o: /opt/ros/electric/ros/core/rosbuild/manifest.xml
CMakeFiles/Serial.dir/src/DrRobotMotionSensorDriver.o: /opt/ros/electric/ros/core/roslang/manifest.xml
CMakeFiles/Serial.dir/src/DrRobotMotionSensorDriver.o: /opt/ros/electric/stacks/ros_comm/utilities/cpp_common/manifest.xml
CMakeFiles/Serial.dir/src/DrRobotMotionSensorDriver.o: /opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp_traits/manifest.xml
CMakeFiles/Serial.dir/src/DrRobotMotionSensorDriver.o: /opt/ros/electric/stacks/ros_comm/utilities/rostime/manifest.xml
CMakeFiles/Serial.dir/src/DrRobotMotionSensorDriver.o: /opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp_serialization/manifest.xml
CMakeFiles/Serial.dir/src/DrRobotMotionSensorDriver.o: /opt/ros/electric/stacks/ros_comm/utilities/xmlrpcpp/manifest.xml
CMakeFiles/Serial.dir/src/DrRobotMotionSensorDriver.o: /opt/ros/electric/stacks/ros_comm/tools/rosconsole/manifest.xml
CMakeFiles/Serial.dir/src/DrRobotMotionSensorDriver.o: /opt/ros/electric/stacks/ros_comm/messages/rosgraph_msgs/manifest.xml
CMakeFiles/Serial.dir/src/DrRobotMotionSensorDriver.o: /opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp/manifest.xml
CMakeFiles/Serial.dir/src/DrRobotMotionSensorDriver.o: /home/jcluck/ros_workspace/ycp-robot/robot_msgs/manifest.xml
CMakeFiles/Serial.dir/src/DrRobotMotionSensorDriver.o: /opt/ros/electric/stacks/ros_comm/messages/std_msgs/msg_gen/generated
CMakeFiles/Serial.dir/src/DrRobotMotionSensorDriver.o: /opt/ros/electric/stacks/ros_comm/messages/rosgraph_msgs/msg_gen/generated
CMakeFiles/Serial.dir/src/DrRobotMotionSensorDriver.o: /opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp/msg_gen/generated
CMakeFiles/Serial.dir/src/DrRobotMotionSensorDriver.o: /opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp/srv_gen/generated
CMakeFiles/Serial.dir/src/DrRobotMotionSensorDriver.o: /home/jcluck/ros_workspace/ycp-robot/robot_msgs/msg_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/jcluck/ros_workspace/ycp-robot/Serial/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/Serial.dir/src/DrRobotMotionSensorDriver.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/Serial.dir/src/DrRobotMotionSensorDriver.o -c /home/jcluck/ros_workspace/ycp-robot/Serial/src/DrRobotMotionSensorDriver.cpp

CMakeFiles/Serial.dir/src/DrRobotMotionSensorDriver.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Serial.dir/src/DrRobotMotionSensorDriver.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/jcluck/ros_workspace/ycp-robot/Serial/src/DrRobotMotionSensorDriver.cpp > CMakeFiles/Serial.dir/src/DrRobotMotionSensorDriver.i

CMakeFiles/Serial.dir/src/DrRobotMotionSensorDriver.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Serial.dir/src/DrRobotMotionSensorDriver.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/jcluck/ros_workspace/ycp-robot/Serial/src/DrRobotMotionSensorDriver.cpp -o CMakeFiles/Serial.dir/src/DrRobotMotionSensorDriver.s

CMakeFiles/Serial.dir/src/DrRobotMotionSensorDriver.o.requires:
.PHONY : CMakeFiles/Serial.dir/src/DrRobotMotionSensorDriver.o.requires

CMakeFiles/Serial.dir/src/DrRobotMotionSensorDriver.o.provides: CMakeFiles/Serial.dir/src/DrRobotMotionSensorDriver.o.requires
	$(MAKE) -f CMakeFiles/Serial.dir/build.make CMakeFiles/Serial.dir/src/DrRobotMotionSensorDriver.o.provides.build
.PHONY : CMakeFiles/Serial.dir/src/DrRobotMotionSensorDriver.o.provides

CMakeFiles/Serial.dir/src/DrRobotMotionSensorDriver.o.provides.build: CMakeFiles/Serial.dir/src/DrRobotMotionSensorDriver.o
.PHONY : CMakeFiles/Serial.dir/src/DrRobotMotionSensorDriver.o.provides.build

# Object files for target Serial
Serial_OBJECTS = \
"CMakeFiles/Serial.dir/src/Converter.o" \
"CMakeFiles/Serial.dir/src/DrRobotMotionSensorDriver.o"

# External object files for target Serial
Serial_EXTERNAL_OBJECTS =

bin/Serial: CMakeFiles/Serial.dir/src/Converter.o
bin/Serial: CMakeFiles/Serial.dir/src/DrRobotMotionSensorDriver.o
bin/Serial: /usr/local/lib/libboost_thread.so
bin/Serial: /usr/local/lib/libboost_signals.so
bin/Serial: CMakeFiles/Serial.dir/build.make
bin/Serial: CMakeFiles/Serial.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable bin/Serial"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Serial.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Serial.dir/build: bin/Serial
.PHONY : CMakeFiles/Serial.dir/build

CMakeFiles/Serial.dir/requires: CMakeFiles/Serial.dir/src/Converter.o.requires
CMakeFiles/Serial.dir/requires: CMakeFiles/Serial.dir/src/DrRobotMotionSensorDriver.o.requires
.PHONY : CMakeFiles/Serial.dir/requires

CMakeFiles/Serial.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Serial.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Serial.dir/clean

CMakeFiles/Serial.dir/depend:
	cd /home/jcluck/ros_workspace/ycp-robot/Serial && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jcluck/ros_workspace/ycp-robot/Serial /home/jcluck/ros_workspace/ycp-robot/Serial /home/jcluck/ros_workspace/ycp-robot/Serial /home/jcluck/ros_workspace/ycp-robot/Serial /home/jcluck/ros_workspace/ycp-robot/Serial/CMakeFiles/Serial.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Serial.dir/depend
