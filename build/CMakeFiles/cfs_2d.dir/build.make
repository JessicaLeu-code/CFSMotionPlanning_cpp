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
CMAKE_SOURCE_DIR = /home/jess/Desktop/GitHub/CFSMotionPlanning_cpp

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jess/Desktop/GitHub/CFSMotionPlanning_cpp/build

# Include any dependencies generated for this target.
include CMakeFiles/cfs_2d.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/cfs_2d.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/cfs_2d.dir/flags.make

CMakeFiles/cfs_2d.dir/src/cfs_2d.cpp.o: CMakeFiles/cfs_2d.dir/flags.make
CMakeFiles/cfs_2d.dir/src/cfs_2d.cpp.o: ../src/cfs_2d.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jess/Desktop/GitHub/CFSMotionPlanning_cpp/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/cfs_2d.dir/src/cfs_2d.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cfs_2d.dir/src/cfs_2d.cpp.o -c /home/jess/Desktop/GitHub/CFSMotionPlanning_cpp/src/cfs_2d.cpp

CMakeFiles/cfs_2d.dir/src/cfs_2d.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cfs_2d.dir/src/cfs_2d.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jess/Desktop/GitHub/CFSMotionPlanning_cpp/src/cfs_2d.cpp > CMakeFiles/cfs_2d.dir/src/cfs_2d.cpp.i

CMakeFiles/cfs_2d.dir/src/cfs_2d.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cfs_2d.dir/src/cfs_2d.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jess/Desktop/GitHub/CFSMotionPlanning_cpp/src/cfs_2d.cpp -o CMakeFiles/cfs_2d.dir/src/cfs_2d.cpp.s

# Object files for target cfs_2d
cfs_2d_OBJECTS = \
"CMakeFiles/cfs_2d.dir/src/cfs_2d.cpp.o"

# External object files for target cfs_2d
cfs_2d_EXTERNAL_OBJECTS =

cfs_2d: CMakeFiles/cfs_2d.dir/src/cfs_2d.cpp.o
cfs_2d: CMakeFiles/cfs_2d.dir/build.make
cfs_2d: src/libquadprog.a
cfs_2d: CMakeFiles/cfs_2d.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jess/Desktop/GitHub/CFSMotionPlanning_cpp/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable cfs_2d"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/cfs_2d.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/cfs_2d.dir/build: cfs_2d

.PHONY : CMakeFiles/cfs_2d.dir/build

CMakeFiles/cfs_2d.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/cfs_2d.dir/cmake_clean.cmake
.PHONY : CMakeFiles/cfs_2d.dir/clean

CMakeFiles/cfs_2d.dir/depend:
	cd /home/jess/Desktop/GitHub/CFSMotionPlanning_cpp/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jess/Desktop/GitHub/CFSMotionPlanning_cpp /home/jess/Desktop/GitHub/CFSMotionPlanning_cpp /home/jess/Desktop/GitHub/CFSMotionPlanning_cpp/build /home/jess/Desktop/GitHub/CFSMotionPlanning_cpp/build /home/jess/Desktop/GitHub/CFSMotionPlanning_cpp/build/CMakeFiles/cfs_2d.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/cfs_2d.dir/depend

