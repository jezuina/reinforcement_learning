# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/jkoroveshi/eclipse-workspace/argos3_examples@argos3-examples-master

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jkoroveshi/eclipse-workspace/argos3_examples@argos3-examples-master

# Utility rule file for mpga_phototaxis_loop_functions_automoc.

# Include the progress variables for this target.
include loop_functions/mpga_loop_functions/CMakeFiles/mpga_phototaxis_loop_functions_automoc.dir/progress.make

loop_functions/mpga_loop_functions/CMakeFiles/mpga_phototaxis_loop_functions_automoc:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/jkoroveshi/eclipse-workspace/argos3_examples@argos3-examples-master/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Automatic moc for target mpga_phototaxis_loop_functions"
	cd /home/jkoroveshi/eclipse-workspace/argos3_examples@argos3-examples-master/loop_functions/mpga_loop_functions && /usr/bin/cmake -E cmake_autogen /home/jkoroveshi/eclipse-workspace/argos3_examples@argos3-examples-master/loop_functions/mpga_loop_functions/CMakeFiles/mpga_phototaxis_loop_functions_automoc.dir/ Debug

mpga_phototaxis_loop_functions_automoc: loop_functions/mpga_loop_functions/CMakeFiles/mpga_phototaxis_loop_functions_automoc
mpga_phototaxis_loop_functions_automoc: loop_functions/mpga_loop_functions/CMakeFiles/mpga_phototaxis_loop_functions_automoc.dir/build.make

.PHONY : mpga_phototaxis_loop_functions_automoc

# Rule to build all files generated by this target.
loop_functions/mpga_loop_functions/CMakeFiles/mpga_phototaxis_loop_functions_automoc.dir/build: mpga_phototaxis_loop_functions_automoc

.PHONY : loop_functions/mpga_loop_functions/CMakeFiles/mpga_phototaxis_loop_functions_automoc.dir/build

loop_functions/mpga_loop_functions/CMakeFiles/mpga_phototaxis_loop_functions_automoc.dir/clean:
	cd /home/jkoroveshi/eclipse-workspace/argos3_examples@argos3-examples-master/loop_functions/mpga_loop_functions && $(CMAKE_COMMAND) -P CMakeFiles/mpga_phototaxis_loop_functions_automoc.dir/cmake_clean.cmake
.PHONY : loop_functions/mpga_loop_functions/CMakeFiles/mpga_phototaxis_loop_functions_automoc.dir/clean

loop_functions/mpga_loop_functions/CMakeFiles/mpga_phototaxis_loop_functions_automoc.dir/depend:
	cd /home/jkoroveshi/eclipse-workspace/argos3_examples@argos3-examples-master && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jkoroveshi/eclipse-workspace/argos3_examples@argos3-examples-master /home/jkoroveshi/eclipse-workspace/argos3_examples@argos3-examples-master/loop_functions/mpga_loop_functions /home/jkoroveshi/eclipse-workspace/argos3_examples@argos3-examples-master /home/jkoroveshi/eclipse-workspace/argos3_examples@argos3-examples-master/loop_functions/mpga_loop_functions /home/jkoroveshi/eclipse-workspace/argos3_examples@argos3-examples-master/loop_functions/mpga_loop_functions/CMakeFiles/mpga_phototaxis_loop_functions_automoc.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : loop_functions/mpga_loop_functions/CMakeFiles/mpga_phototaxis_loop_functions_automoc.dir/depend

