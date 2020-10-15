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
CMAKE_SOURCE_DIR = /home/leo/leo_slam/src/tracking_slam/libsegmentation

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/leo/leo_slam/src/tracking_slam/libsegmentation/build

# Include any dependencies generated for this target.
include CMakeFiles/segmentation.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/segmentation.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/segmentation.dir/flags.make

CMakeFiles/segmentation.dir/libsegmentation.cpp.o: CMakeFiles/segmentation.dir/flags.make
CMakeFiles/segmentation.dir/libsegmentation.cpp.o: ../libsegmentation.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leo/leo_slam/src/tracking_slam/libsegmentation/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/segmentation.dir/libsegmentation.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/segmentation.dir/libsegmentation.cpp.o -c /home/leo/leo_slam/src/tracking_slam/libsegmentation/libsegmentation.cpp

CMakeFiles/segmentation.dir/libsegmentation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/segmentation.dir/libsegmentation.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leo/leo_slam/src/tracking_slam/libsegmentation/libsegmentation.cpp > CMakeFiles/segmentation.dir/libsegmentation.cpp.i

CMakeFiles/segmentation.dir/libsegmentation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/segmentation.dir/libsegmentation.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leo/leo_slam/src/tracking_slam/libsegmentation/libsegmentation.cpp -o CMakeFiles/segmentation.dir/libsegmentation.cpp.s

CMakeFiles/segmentation.dir/libsegmentation.cpp.o.requires:

.PHONY : CMakeFiles/segmentation.dir/libsegmentation.cpp.o.requires

CMakeFiles/segmentation.dir/libsegmentation.cpp.o.provides: CMakeFiles/segmentation.dir/libsegmentation.cpp.o.requires
	$(MAKE) -f CMakeFiles/segmentation.dir/build.make CMakeFiles/segmentation.dir/libsegmentation.cpp.o.provides.build
.PHONY : CMakeFiles/segmentation.dir/libsegmentation.cpp.o.provides

CMakeFiles/segmentation.dir/libsegmentation.cpp.o.provides.build: CMakeFiles/segmentation.dir/libsegmentation.cpp.o


# Object files for target segmentation
segmentation_OBJECTS = \
"CMakeFiles/segmentation.dir/libsegmentation.cpp.o"

# External object files for target segmentation
segmentation_EXTERNAL_OBJECTS =

libsegmentation.so: CMakeFiles/segmentation.dir/libsegmentation.cpp.o
libsegmentation.so: CMakeFiles/segmentation.dir/build.make
libsegmentation.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stitching3.so.3.3.1
libsegmentation.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_superres3.so.3.3.1
libsegmentation.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videostab3.so.3.3.1
libsegmentation.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_aruco3.so.3.3.1
libsegmentation.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bgsegm3.so.3.3.1
libsegmentation.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bioinspired3.so.3.3.1
libsegmentation.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ccalib3.so.3.3.1
libsegmentation.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_cvv3.so.3.3.1
libsegmentation.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dpm3.so.3.3.1
libsegmentation.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_face3.so.3.3.1
libsegmentation.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_fuzzy3.so.3.3.1
libsegmentation.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_hdf3.so.3.3.1
libsegmentation.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_img_hash3.so.3.3.1
libsegmentation.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_line_descriptor3.so.3.3.1
libsegmentation.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_optflow3.so.3.3.1
libsegmentation.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_reg3.so.3.3.1
libsegmentation.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_rgbd3.so.3.3.1
libsegmentation.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_saliency3.so.3.3.1
libsegmentation.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stereo3.so.3.3.1
libsegmentation.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_structured_light3.so.3.3.1
libsegmentation.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_surface_matching3.so.3.3.1
libsegmentation.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_tracking3.so.3.3.1
libsegmentation.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xfeatures2d3.so.3.3.1
libsegmentation.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ximgproc3.so.3.3.1
libsegmentation.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xobjdetect3.so.3.3.1
libsegmentation.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xphoto3.so.3.3.1
libsegmentation.so: ../../caffe-segnet-cudnn5/build/lib/libcaffe.so
libsegmentation.so: /usr/lib/x86_64-linux-gnu/libglog.so
libsegmentation.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_shape3.so.3.3.1
libsegmentation.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_photo3.so.3.3.1
libsegmentation.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_datasets3.so.3.3.1
libsegmentation.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_plot3.so.3.3.1
libsegmentation.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_text3.so.3.3.1
libsegmentation.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dnn3.so.3.3.1
libsegmentation.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ml3.so.3.3.1
libsegmentation.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_video3.so.3.3.1
libsegmentation.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_calib3d3.so.3.3.1
libsegmentation.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_features2d3.so.3.3.1
libsegmentation.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_highgui3.so.3.3.1
libsegmentation.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videoio3.so.3.3.1
libsegmentation.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_viz3.so.3.3.1
libsegmentation.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_phase_unwrapping3.so.3.3.1
libsegmentation.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_flann3.so.3.3.1
libsegmentation.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgcodecs3.so.3.3.1
libsegmentation.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_objdetect3.so.3.3.1
libsegmentation.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgproc3.so.3.3.1
libsegmentation.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_core3.so.3.3.1
libsegmentation.so: CMakeFiles/segmentation.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/leo/leo_slam/src/tracking_slam/libsegmentation/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libsegmentation.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/segmentation.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/segmentation.dir/build: libsegmentation.so

.PHONY : CMakeFiles/segmentation.dir/build

CMakeFiles/segmentation.dir/requires: CMakeFiles/segmentation.dir/libsegmentation.cpp.o.requires

.PHONY : CMakeFiles/segmentation.dir/requires

CMakeFiles/segmentation.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/segmentation.dir/cmake_clean.cmake
.PHONY : CMakeFiles/segmentation.dir/clean

CMakeFiles/segmentation.dir/depend:
	cd /home/leo/leo_slam/src/tracking_slam/libsegmentation/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/leo/leo_slam/src/tracking_slam/libsegmentation /home/leo/leo_slam/src/tracking_slam/libsegmentation /home/leo/leo_slam/src/tracking_slam/libsegmentation/build /home/leo/leo_slam/src/tracking_slam/libsegmentation/build /home/leo/leo_slam/src/tracking_slam/libsegmentation/build/CMakeFiles/segmentation.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/segmentation.dir/depend

