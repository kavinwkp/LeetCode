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
CMAKE_SOURCE_DIR = /home/kavin/algorithm/LeetCode/OpenCV/test01

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kavin/algorithm/LeetCode/OpenCV/test01/build

# Include any dependencies generated for this target.
include CMakeFiles/main1.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/main1.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/main1.dir/flags.make

CMakeFiles/main1.dir/main1.cpp.o: CMakeFiles/main1.dir/flags.make
CMakeFiles/main1.dir/main1.cpp.o: ../main1.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kavin/algorithm/LeetCode/OpenCV/test01/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/main1.dir/main1.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main1.dir/main1.cpp.o -c /home/kavin/algorithm/LeetCode/OpenCV/test01/main1.cpp

CMakeFiles/main1.dir/main1.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main1.dir/main1.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kavin/algorithm/LeetCode/OpenCV/test01/main1.cpp > CMakeFiles/main1.dir/main1.cpp.i

CMakeFiles/main1.dir/main1.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main1.dir/main1.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kavin/algorithm/LeetCode/OpenCV/test01/main1.cpp -o CMakeFiles/main1.dir/main1.cpp.s

CMakeFiles/main1.dir/main1.cpp.o.requires:

.PHONY : CMakeFiles/main1.dir/main1.cpp.o.requires

CMakeFiles/main1.dir/main1.cpp.o.provides: CMakeFiles/main1.dir/main1.cpp.o.requires
	$(MAKE) -f CMakeFiles/main1.dir/build.make CMakeFiles/main1.dir/main1.cpp.o.provides.build
.PHONY : CMakeFiles/main1.dir/main1.cpp.o.provides

CMakeFiles/main1.dir/main1.cpp.o.provides.build: CMakeFiles/main1.dir/main1.cpp.o


# Object files for target main1
main1_OBJECTS = \
"CMakeFiles/main1.dir/main1.cpp.o"

# External object files for target main1
main1_EXTERNAL_OBJECTS =

main1: CMakeFiles/main1.dir/main1.cpp.o
main1: CMakeFiles/main1.dir/build.make
main1: /home/kavin/Library/opencv-3.3.1/build/installed/lib/libopencv_stitching.so.3.3.1
main1: /home/kavin/Library/opencv-3.3.1/build/installed/lib/libopencv_superres.so.3.3.1
main1: /home/kavin/Library/opencv-3.3.1/build/installed/lib/libopencv_videostab.so.3.3.1
main1: /home/kavin/Library/opencv-3.3.1/build/installed/lib/libopencv_aruco.so.3.3.1
main1: /home/kavin/Library/opencv-3.3.1/build/installed/lib/libopencv_bgsegm.so.3.3.1
main1: /home/kavin/Library/opencv-3.3.1/build/installed/lib/libopencv_bioinspired.so.3.3.1
main1: /home/kavin/Library/opencv-3.3.1/build/installed/lib/libopencv_ccalib.so.3.3.1
main1: /home/kavin/Library/opencv-3.3.1/build/installed/lib/libopencv_dpm.so.3.3.1
main1: /home/kavin/Library/opencv-3.3.1/build/installed/lib/libopencv_face.so.3.3.1
main1: /home/kavin/Library/opencv-3.3.1/build/installed/lib/libopencv_freetype.so.3.3.1
main1: /home/kavin/Library/opencv-3.3.1/build/installed/lib/libopencv_fuzzy.so.3.3.1
main1: /home/kavin/Library/opencv-3.3.1/build/installed/lib/libopencv_hdf.so.3.3.1
main1: /home/kavin/Library/opencv-3.3.1/build/installed/lib/libopencv_img_hash.so.3.3.1
main1: /home/kavin/Library/opencv-3.3.1/build/installed/lib/libopencv_line_descriptor.so.3.3.1
main1: /home/kavin/Library/opencv-3.3.1/build/installed/lib/libopencv_optflow.so.3.3.1
main1: /home/kavin/Library/opencv-3.3.1/build/installed/lib/libopencv_reg.so.3.3.1
main1: /home/kavin/Library/opencv-3.3.1/build/installed/lib/libopencv_rgbd.so.3.3.1
main1: /home/kavin/Library/opencv-3.3.1/build/installed/lib/libopencv_saliency.so.3.3.1
main1: /home/kavin/Library/opencv-3.3.1/build/installed/lib/libopencv_sfm.so.3.3.1
main1: /home/kavin/Library/opencv-3.3.1/build/installed/lib/libopencv_stereo.so.3.3.1
main1: /home/kavin/Library/opencv-3.3.1/build/installed/lib/libopencv_structured_light.so.3.3.1
main1: /home/kavin/Library/opencv-3.3.1/build/installed/lib/libopencv_surface_matching.so.3.3.1
main1: /home/kavin/Library/opencv-3.3.1/build/installed/lib/libopencv_tracking.so.3.3.1
main1: /home/kavin/Library/opencv-3.3.1/build/installed/lib/libopencv_xfeatures2d.so.3.3.1
main1: /home/kavin/Library/opencv-3.3.1/build/installed/lib/libopencv_ximgproc.so.3.3.1
main1: /home/kavin/Library/opencv-3.3.1/build/installed/lib/libopencv_xobjdetect.so.3.3.1
main1: /home/kavin/Library/opencv-3.3.1/build/installed/lib/libopencv_xphoto.so.3.3.1
main1: /home/kavin/Library/opencv-3.3.1/build/installed/lib/libopencv_photo.so.3.3.1
main1: /home/kavin/Library/opencv-3.3.1/build/installed/lib/libopencv_shape.so.3.3.1
main1: /home/kavin/Library/opencv-3.3.1/build/installed/lib/libopencv_calib3d.so.3.3.1
main1: /home/kavin/Library/opencv-3.3.1/build/installed/lib/libopencv_viz.so.3.3.1
main1: /home/kavin/Library/opencv-3.3.1/build/installed/lib/libopencv_phase_unwrapping.so.3.3.1
main1: /home/kavin/Library/opencv-3.3.1/build/installed/lib/libopencv_video.so.3.3.1
main1: /home/kavin/Library/opencv-3.3.1/build/installed/lib/libopencv_datasets.so.3.3.1
main1: /home/kavin/Library/opencv-3.3.1/build/installed/lib/libopencv_plot.so.3.3.1
main1: /home/kavin/Library/opencv-3.3.1/build/installed/lib/libopencv_text.so.3.3.1
main1: /home/kavin/Library/opencv-3.3.1/build/installed/lib/libopencv_dnn.so.3.3.1
main1: /home/kavin/Library/opencv-3.3.1/build/installed/lib/libopencv_features2d.so.3.3.1
main1: /home/kavin/Library/opencv-3.3.1/build/installed/lib/libopencv_flann.so.3.3.1
main1: /home/kavin/Library/opencv-3.3.1/build/installed/lib/libopencv_highgui.so.3.3.1
main1: /home/kavin/Library/opencv-3.3.1/build/installed/lib/libopencv_ml.so.3.3.1
main1: /home/kavin/Library/opencv-3.3.1/build/installed/lib/libopencv_videoio.so.3.3.1
main1: /home/kavin/Library/opencv-3.3.1/build/installed/lib/libopencv_imgcodecs.so.3.3.1
main1: /home/kavin/Library/opencv-3.3.1/build/installed/lib/libopencv_objdetect.so.3.3.1
main1: /home/kavin/Library/opencv-3.3.1/build/installed/lib/libopencv_imgproc.so.3.3.1
main1: /home/kavin/Library/opencv-3.3.1/build/installed/lib/libopencv_core.so.3.3.1
main1: CMakeFiles/main1.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kavin/algorithm/LeetCode/OpenCV/test01/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable main1"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/main1.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/main1.dir/build: main1

.PHONY : CMakeFiles/main1.dir/build

CMakeFiles/main1.dir/requires: CMakeFiles/main1.dir/main1.cpp.o.requires

.PHONY : CMakeFiles/main1.dir/requires

CMakeFiles/main1.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/main1.dir/cmake_clean.cmake
.PHONY : CMakeFiles/main1.dir/clean

CMakeFiles/main1.dir/depend:
	cd /home/kavin/algorithm/LeetCode/OpenCV/test01/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kavin/algorithm/LeetCode/OpenCV/test01 /home/kavin/algorithm/LeetCode/OpenCV/test01 /home/kavin/algorithm/LeetCode/OpenCV/test01/build /home/kavin/algorithm/LeetCode/OpenCV/test01/build /home/kavin/algorithm/LeetCode/OpenCV/test01/build/CMakeFiles/main1.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/main1.dir/depend
