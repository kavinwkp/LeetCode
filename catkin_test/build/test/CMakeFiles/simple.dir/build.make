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
CMAKE_SOURCE_DIR = /home/kavin/ROSProjects/catkin_test/Modules/test

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kavin/ROSProjects/catkin_test/build/test

# Include any dependencies generated for this target.
include CMakeFiles/simple.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/simple.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/simple.dir/flags.make

CMakeFiles/simple.dir/src/simple.cpp.o: CMakeFiles/simple.dir/flags.make
CMakeFiles/simple.dir/src/simple.cpp.o: /home/kavin/ROSProjects/catkin_test/Modules/test/src/simple.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kavin/ROSProjects/catkin_test/build/test/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/simple.dir/src/simple.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/simple.dir/src/simple.cpp.o -c /home/kavin/ROSProjects/catkin_test/Modules/test/src/simple.cpp

CMakeFiles/simple.dir/src/simple.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/simple.dir/src/simple.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kavin/ROSProjects/catkin_test/Modules/test/src/simple.cpp > CMakeFiles/simple.dir/src/simple.cpp.i

CMakeFiles/simple.dir/src/simple.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/simple.dir/src/simple.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kavin/ROSProjects/catkin_test/Modules/test/src/simple.cpp -o CMakeFiles/simple.dir/src/simple.cpp.s

CMakeFiles/simple.dir/src/simple.cpp.o.requires:

.PHONY : CMakeFiles/simple.dir/src/simple.cpp.o.requires

CMakeFiles/simple.dir/src/simple.cpp.o.provides: CMakeFiles/simple.dir/src/simple.cpp.o.requires
	$(MAKE) -f CMakeFiles/simple.dir/build.make CMakeFiles/simple.dir/src/simple.cpp.o.provides.build
.PHONY : CMakeFiles/simple.dir/src/simple.cpp.o.provides

CMakeFiles/simple.dir/src/simple.cpp.o.provides.build: CMakeFiles/simple.dir/src/simple.cpp.o


# Object files for target simple
simple_OBJECTS = \
"CMakeFiles/simple.dir/src/simple.cpp.o"

# External object files for target simple
simple_EXTERNAL_OBJECTS =

/home/kavin/ROSProjects/catkin_test/devel/lib/libsimple.so: CMakeFiles/simple.dir/src/simple.cpp.o
/home/kavin/ROSProjects/catkin_test/devel/lib/libsimple.so: CMakeFiles/simple.dir/build.make
/home/kavin/ROSProjects/catkin_test/devel/lib/libsimple.so: CMakeFiles/simple.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kavin/ROSProjects/catkin_test/build/test/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/kavin/ROSProjects/catkin_test/devel/lib/libsimple.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/simple.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/simple.dir/build: /home/kavin/ROSProjects/catkin_test/devel/lib/libsimple.so

.PHONY : CMakeFiles/simple.dir/build

CMakeFiles/simple.dir/requires: CMakeFiles/simple.dir/src/simple.cpp.o.requires

.PHONY : CMakeFiles/simple.dir/requires

CMakeFiles/simple.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/simple.dir/cmake_clean.cmake
.PHONY : CMakeFiles/simple.dir/clean

CMakeFiles/simple.dir/depend:
	cd /home/kavin/ROSProjects/catkin_test/build/test && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kavin/ROSProjects/catkin_test/Modules/test /home/kavin/ROSProjects/catkin_test/Modules/test /home/kavin/ROSProjects/catkin_test/build/test /home/kavin/ROSProjects/catkin_test/build/test /home/kavin/ROSProjects/catkin_test/build/test/CMakeFiles/simple.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/simple.dir/depend

