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
CMAKE_SOURCE_DIR = /home/jkoroveshi/argos3-examples-master

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jkoroveshi/argos3-examples-master

# Utility rule file for argos3_automoc.

# Include the progress variables for this target.
include CMakeFiles/argos3_automoc.dir/progress.make

CMakeFiles/argos3_automoc:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/jkoroveshi/argos3-examples-master/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Automatic moc for target argos3"
	/usr/bin/cmake -E cmake_autogen /home/jkoroveshi/argos3-examples-master/CMakeFiles/argos3_automoc.dir/ Debug

argos3_automoc: CMakeFiles/argos3_automoc
argos3_automoc: CMakeFiles/argos3_automoc.dir/build.make

.PHONY : argos3_automoc

# Rule to build all files generated by this target.
CMakeFiles/argos3_automoc.dir/build: argos3_automoc

.PHONY : CMakeFiles/argos3_automoc.dir/build

CMakeFiles/argos3_automoc.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/argos3_automoc.dir/cmake_clean.cmake
.PHONY : CMakeFiles/argos3_automoc.dir/clean

CMakeFiles/argos3_automoc.dir/depend:
	cd /home/jkoroveshi/argos3-examples-master && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jkoroveshi/argos3-examples-master /home/jkoroveshi/argos3-examples-master /home/jkoroveshi/argos3-examples-master /home/jkoroveshi/argos3-examples-master /home/jkoroveshi/argos3-examples-master/CMakeFiles/argos3_automoc.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/argos3_automoc.dir/depend

