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
CMAKE_SOURCE_DIR = /home/dwicke/git/ycp-robot/sim

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/dwicke/git/ycp-robot/sim/build

# Include any dependencies generated for this target.
include CMakeFiles/simtest.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/simtest.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/simtest.dir/flags.make

CMakeFiles/simtest.dir/src/simtest.o: CMakeFiles/simtest.dir/flags.make
CMakeFiles/simtest.dir/src/simtest.o: ../src/simtest.cpp
CMakeFiles/simtest.dir/src/simtest.o: ../manifest.xml
CMakeFiles/simtest.dir/src/simtest.o: /opt/ros/electric/ros/tools/rospack/manifest.xml
CMakeFiles/simtest.dir/src/simtest.o: /opt/ros/electric/ros/core/roslib/manifest.xml
CMakeFiles/simtest.dir/src/simtest.o: /opt/ros/electric/stacks/ros_comm/messages/std_msgs/manifest.xml
CMakeFiles/simtest.dir/src/simtest.o: /opt/ros/electric/ros/core/rosbuild/manifest.xml
CMakeFiles/simtest.dir/src/simtest.o: /opt/ros/electric/ros/core/roslang/manifest.xml
CMakeFiles/simtest.dir/src/simtest.o: /opt/ros/electric/stacks/ros_comm/utilities/cpp_common/manifest.xml
CMakeFiles/simtest.dir/src/simtest.o: /opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp_traits/manifest.xml
CMakeFiles/simtest.dir/src/simtest.o: /opt/ros/electric/stacks/ros_comm/utilities/rostime/manifest.xml
CMakeFiles/simtest.dir/src/simtest.o: /opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp_serialization/manifest.xml
CMakeFiles/simtest.dir/src/simtest.o: /opt/ros/electric/stacks/ros_comm/utilities/xmlrpcpp/manifest.xml
CMakeFiles/simtest.dir/src/simtest.o: /opt/ros/electric/stacks/ros_comm/tools/rosconsole/manifest.xml
CMakeFiles/simtest.dir/src/simtest.o: /opt/ros/electric/stacks/ros_comm/messages/rosgraph_msgs/manifest.xml
CMakeFiles/simtest.dir/src/simtest.o: /opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp/manifest.xml
CMakeFiles/simtest.dir/src/simtest.o: /home/dwicke/git/ycp-robot/robot_msgs/manifest.xml
CMakeFiles/simtest.dir/src/simtest.o: /opt/ros/electric/stacks/ros_comm/messages/std_msgs/msg_gen/generated
CMakeFiles/simtest.dir/src/simtest.o: /opt/ros/electric/stacks/ros_comm/messages/rosgraph_msgs/msg_gen/generated
CMakeFiles/simtest.dir/src/simtest.o: /opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp/msg_gen/generated
CMakeFiles/simtest.dir/src/simtest.o: /opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp/srv_gen/generated
CMakeFiles/simtest.dir/src/simtest.o: /home/dwicke/git/ycp-robot/robot_msgs/msg_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/dwicke/git/ycp-robot/sim/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/simtest.dir/src/simtest.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/simtest.dir/src/simtest.o -c /home/dwicke/git/ycp-robot/sim/src/simtest.cpp

CMakeFiles/simtest.dir/src/simtest.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/simtest.dir/src/simtest.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/dwicke/git/ycp-robot/sim/src/simtest.cpp > CMakeFiles/simtest.dir/src/simtest.i

CMakeFiles/simtest.dir/src/simtest.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/simtest.dir/src/simtest.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/dwicke/git/ycp-robot/sim/src/simtest.cpp -o CMakeFiles/simtest.dir/src/simtest.s

CMakeFiles/simtest.dir/src/simtest.o.requires:
.PHONY : CMakeFiles/simtest.dir/src/simtest.o.requires

CMakeFiles/simtest.dir/src/simtest.o.provides: CMakeFiles/simtest.dir/src/simtest.o.requires
	$(MAKE) -f CMakeFiles/simtest.dir/build.make CMakeFiles/simtest.dir/src/simtest.o.provides.build
.PHONY : CMakeFiles/simtest.dir/src/simtest.o.provides

CMakeFiles/simtest.dir/src/simtest.o.provides.build: CMakeFiles/simtest.dir/src/simtest.o
.PHONY : CMakeFiles/simtest.dir/src/simtest.o.provides.build

# Object files for target simtest
simtest_OBJECTS = \
"CMakeFiles/simtest.dir/src/simtest.o"

# External object files for target simtest
simtest_EXTERNAL_OBJECTS =

../bin/simtest: CMakeFiles/simtest.dir/src/simtest.o
../bin/simtest: CMakeFiles/simtest.dir/build.make
../bin/simtest: CMakeFiles/simtest.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/simtest"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/simtest.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/simtest.dir/build: ../bin/simtest
.PHONY : CMakeFiles/simtest.dir/build

CMakeFiles/simtest.dir/requires: CMakeFiles/simtest.dir/src/simtest.o.requires
.PHONY : CMakeFiles/simtest.dir/requires

CMakeFiles/simtest.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/simtest.dir/cmake_clean.cmake
.PHONY : CMakeFiles/simtest.dir/clean

CMakeFiles/simtest.dir/depend:
	cd /home/dwicke/git/ycp-robot/sim/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dwicke/git/ycp-robot/sim /home/dwicke/git/ycp-robot/sim /home/dwicke/git/ycp-robot/sim/build /home/dwicke/git/ycp-robot/sim/build /home/dwicke/git/ycp-robot/sim/build/CMakeFiles/simtest.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/simtest.dir/depend

