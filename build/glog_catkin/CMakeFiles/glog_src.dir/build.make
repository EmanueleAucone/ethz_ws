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
CMAKE_SOURCE_DIR = /home/emanuele/ethz_ws/src/glog_catkin

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/emanuele/ethz_ws/build/glog_catkin

# Utility rule file for glog_src.

# Include the progress variables for this target.
include CMakeFiles/glog_src.dir/progress.make

CMakeFiles/glog_src: CMakeFiles/glog_src-complete


CMakeFiles/glog_src-complete: glog_src-prefix/src/glog_src-stamp/glog_src-install
CMakeFiles/glog_src-complete: glog_src-prefix/src/glog_src-stamp/glog_src-mkdir
CMakeFiles/glog_src-complete: glog_src-prefix/src/glog_src-stamp/glog_src-download
CMakeFiles/glog_src-complete: glog_src-prefix/src/glog_src-stamp/glog_src-update
CMakeFiles/glog_src-complete: glog_src-prefix/src/glog_src-stamp/glog_src-patch
CMakeFiles/glog_src-complete: glog_src-prefix/src/glog_src-stamp/glog_src-configure
CMakeFiles/glog_src-complete: glog_src-prefix/src/glog_src-stamp/glog_src-build
CMakeFiles/glog_src-complete: glog_src-prefix/src/glog_src-stamp/glog_src-install
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/emanuele/ethz_ws/build/glog_catkin/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Completed 'glog_src'"
	/usr/bin/cmake -E make_directory /home/emanuele/ethz_ws/build/glog_catkin/CMakeFiles
	/usr/bin/cmake -E touch /home/emanuele/ethz_ws/build/glog_catkin/CMakeFiles/glog_src-complete
	/usr/bin/cmake -E touch /home/emanuele/ethz_ws/build/glog_catkin/glog_src-prefix/src/glog_src-stamp/glog_src-done

glog_src-prefix/src/glog_src-stamp/glog_src-install: glog_src-prefix/src/glog_src-stamp/glog_src-build
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/emanuele/ethz_ws/build/glog_catkin/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Performing install step for 'glog_src'"
	cd /home/emanuele/ethz_ws/build/glog_catkin/glog_src-prefix/src/glog_src-build && cd ../glog_src/ && make install -j 8
	cd /home/emanuele/ethz_ws/build/glog_catkin/glog_src-prefix/src/glog_src-build && /usr/bin/cmake -E touch /home/emanuele/ethz_ws/build/glog_catkin/glog_src-prefix/src/glog_src-stamp/glog_src-install

glog_src-prefix/src/glog_src-stamp/glog_src-mkdir:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/emanuele/ethz_ws/build/glog_catkin/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Creating directories for 'glog_src'"
	/usr/bin/cmake -E make_directory /home/emanuele/ethz_ws/build/glog_catkin/glog_src-prefix/src/glog_src
	/usr/bin/cmake -E make_directory /home/emanuele/ethz_ws/build/glog_catkin/glog_src-prefix/src/glog_src-build
	/usr/bin/cmake -E make_directory /home/emanuele/ethz_ws/build/glog_catkin/glog_src-prefix
	/usr/bin/cmake -E make_directory /home/emanuele/ethz_ws/build/glog_catkin/glog_src-prefix/tmp
	/usr/bin/cmake -E make_directory /home/emanuele/ethz_ws/build/glog_catkin/glog_src-prefix/src/glog_src-stamp
	/usr/bin/cmake -E make_directory /home/emanuele/ethz_ws/build/glog_catkin/glog_src-prefix/src
	/usr/bin/cmake -E touch /home/emanuele/ethz_ws/build/glog_catkin/glog_src-prefix/src/glog_src-stamp/glog_src-mkdir

glog_src-prefix/src/glog_src-stamp/glog_src-download: glog_src-prefix/src/glog_src-stamp/glog_src-urlinfo.txt
glog_src-prefix/src/glog_src-stamp/glog_src-download: glog_src-prefix/src/glog_src-stamp/glog_src-mkdir
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/emanuele/ethz_ws/build/glog_catkin/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Performing download step (download, verify and extract) for 'glog_src'"
	cd /home/emanuele/ethz_ws/build/glog_catkin/glog_src-prefix/src && /usr/bin/cmake -P /home/emanuele/ethz_ws/build/glog_catkin/glog_src-prefix/src/glog_src-stamp/download-glog_src.cmake
	cd /home/emanuele/ethz_ws/build/glog_catkin/glog_src-prefix/src && /usr/bin/cmake -P /home/emanuele/ethz_ws/build/glog_catkin/glog_src-prefix/src/glog_src-stamp/verify-glog_src.cmake
	cd /home/emanuele/ethz_ws/build/glog_catkin/glog_src-prefix/src && /usr/bin/cmake -P /home/emanuele/ethz_ws/build/glog_catkin/glog_src-prefix/src/glog_src-stamp/extract-glog_src.cmake
	cd /home/emanuele/ethz_ws/build/glog_catkin/glog_src-prefix/src && /usr/bin/cmake -E touch /home/emanuele/ethz_ws/build/glog_catkin/glog_src-prefix/src/glog_src-stamp/glog_src-download

glog_src-prefix/src/glog_src-stamp/glog_src-update: glog_src-prefix/src/glog_src-stamp/glog_src-download
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/emanuele/ethz_ws/build/glog_catkin/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "No update step for 'glog_src'"
	cd /home/emanuele/ethz_ws/build/glog_catkin/glog_src-prefix/src/glog_src && /usr/bin/cmake -E echo_append
	cd /home/emanuele/ethz_ws/build/glog_catkin/glog_src-prefix/src/glog_src && /usr/bin/cmake -E touch /home/emanuele/ethz_ws/build/glog_catkin/glog_src-prefix/src/glog_src-stamp/glog_src-update

glog_src-prefix/src/glog_src-stamp/glog_src-patch: glog_src-prefix/src/glog_src-stamp/glog_src-download
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/emanuele/ethz_ws/build/glog_catkin/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Performing patch step for 'glog_src'"
	cd /home/emanuele/ethz_ws/build/glog_catkin/glog_src-prefix/src/glog_src && patch -p0 < /home/emanuele/ethz_ws/src/glog_catkin/fix-unused-typedef-warning.patch
	cd /home/emanuele/ethz_ws/build/glog_catkin/glog_src-prefix/src/glog_src && /usr/bin/cmake -E touch /home/emanuele/ethz_ws/build/glog_catkin/glog_src-prefix/src/glog_src-stamp/glog_src-patch

glog_src-prefix/src/glog_src-stamp/glog_src-configure: glog_src-prefix/tmp/glog_src-cfgcmd.txt
glog_src-prefix/src/glog_src-stamp/glog_src-configure: glog_src-prefix/src/glog_src-stamp/glog_src-update
glog_src-prefix/src/glog_src-stamp/glog_src-configure: glog_src-prefix/src/glog_src-stamp/glog_src-patch
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/emanuele/ethz_ws/build/glog_catkin/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Performing configure step for 'glog_src'"
	cd /home/emanuele/ethz_ws/build/glog_catkin/glog_src-prefix/src/glog_src-build && cd ../glog_src/ && autoreconf -fi && ./configure --with-pic --with-gflags= --prefix=/home/emanuele/ethz_ws/devel/.private/glog_catkin
	cd /home/emanuele/ethz_ws/build/glog_catkin/glog_src-prefix/src/glog_src-build && /usr/bin/cmake -E touch /home/emanuele/ethz_ws/build/glog_catkin/glog_src-prefix/src/glog_src-stamp/glog_src-configure

glog_src-prefix/src/glog_src-stamp/glog_src-build: glog_src-prefix/src/glog_src-stamp/glog_src-configure
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/emanuele/ethz_ws/build/glog_catkin/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Performing build step for 'glog_src'"
	cd /home/emanuele/ethz_ws/build/glog_catkin/glog_src-prefix/src/glog_src-build && cd ../glog_src/ && make -j 8
	cd /home/emanuele/ethz_ws/build/glog_catkin/glog_src-prefix/src/glog_src-build && /usr/bin/cmake -E touch /home/emanuele/ethz_ws/build/glog_catkin/glog_src-prefix/src/glog_src-stamp/glog_src-build

glog_src: CMakeFiles/glog_src
glog_src: CMakeFiles/glog_src-complete
glog_src: glog_src-prefix/src/glog_src-stamp/glog_src-install
glog_src: glog_src-prefix/src/glog_src-stamp/glog_src-mkdir
glog_src: glog_src-prefix/src/glog_src-stamp/glog_src-download
glog_src: glog_src-prefix/src/glog_src-stamp/glog_src-update
glog_src: glog_src-prefix/src/glog_src-stamp/glog_src-patch
glog_src: glog_src-prefix/src/glog_src-stamp/glog_src-configure
glog_src: glog_src-prefix/src/glog_src-stamp/glog_src-build
glog_src: CMakeFiles/glog_src.dir/build.make

.PHONY : glog_src

# Rule to build all files generated by this target.
CMakeFiles/glog_src.dir/build: glog_src

.PHONY : CMakeFiles/glog_src.dir/build

CMakeFiles/glog_src.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/glog_src.dir/cmake_clean.cmake
.PHONY : CMakeFiles/glog_src.dir/clean

CMakeFiles/glog_src.dir/depend:
	cd /home/emanuele/ethz_ws/build/glog_catkin && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/emanuele/ethz_ws/src/glog_catkin /home/emanuele/ethz_ws/src/glog_catkin /home/emanuele/ethz_ws/build/glog_catkin /home/emanuele/ethz_ws/build/glog_catkin /home/emanuele/ethz_ws/build/glog_catkin/CMakeFiles/glog_src.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/glog_src.dir/depend

