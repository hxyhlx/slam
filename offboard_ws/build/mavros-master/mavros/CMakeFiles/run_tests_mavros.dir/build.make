# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/xhe/offboard_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/xhe/offboard_ws/build

# Utility rule file for run_tests_mavros.

# Include the progress variables for this target.
include mavros-master/mavros/CMakeFiles/run_tests_mavros.dir/progress.make

run_tests_mavros: mavros-master/mavros/CMakeFiles/run_tests_mavros.dir/build.make

.PHONY : run_tests_mavros

# Rule to build all files generated by this target.
mavros-master/mavros/CMakeFiles/run_tests_mavros.dir/build: run_tests_mavros

.PHONY : mavros-master/mavros/CMakeFiles/run_tests_mavros.dir/build

mavros-master/mavros/CMakeFiles/run_tests_mavros.dir/clean:
	cd /home/xhe/offboard_ws/build/mavros-master/mavros && $(CMAKE_COMMAND) -P CMakeFiles/run_tests_mavros.dir/cmake_clean.cmake
.PHONY : mavros-master/mavros/CMakeFiles/run_tests_mavros.dir/clean

mavros-master/mavros/CMakeFiles/run_tests_mavros.dir/depend:
	cd /home/xhe/offboard_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/xhe/offboard_ws/src /home/xhe/offboard_ws/src/mavros-master/mavros /home/xhe/offboard_ws/build /home/xhe/offboard_ws/build/mavros-master/mavros /home/xhe/offboard_ws/build/mavros-master/mavros/CMakeFiles/run_tests_mavros.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : mavros-master/mavros/CMakeFiles/run_tests_mavros.dir/depend

