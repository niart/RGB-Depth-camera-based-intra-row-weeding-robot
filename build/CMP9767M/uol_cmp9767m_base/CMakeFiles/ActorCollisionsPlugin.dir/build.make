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
CMAKE_SOURCE_DIR = /home/bonjour/NW_assignment_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/bonjour/NW_assignment_ws/build

# Include any dependencies generated for this target.
include CMP9767M/uol_cmp9767m_base/CMakeFiles/ActorCollisionsPlugin.dir/depend.make

# Include the progress variables for this target.
include CMP9767M/uol_cmp9767m_base/CMakeFiles/ActorCollisionsPlugin.dir/progress.make

# Include the compile flags for this target's objects.
include CMP9767M/uol_cmp9767m_base/CMakeFiles/ActorCollisionsPlugin.dir/flags.make

CMP9767M/uol_cmp9767m_base/CMakeFiles/ActorCollisionsPlugin.dir/src/ActorCollisionsPlugin.cc.o: CMP9767M/uol_cmp9767m_base/CMakeFiles/ActorCollisionsPlugin.dir/flags.make
CMP9767M/uol_cmp9767m_base/CMakeFiles/ActorCollisionsPlugin.dir/src/ActorCollisionsPlugin.cc.o: /home/bonjour/NW_assignment_ws/src/CMP9767M/uol_cmp9767m_base/src/ActorCollisionsPlugin.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bonjour/NW_assignment_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMP9767M/uol_cmp9767m_base/CMakeFiles/ActorCollisionsPlugin.dir/src/ActorCollisionsPlugin.cc.o"
	cd /home/bonjour/NW_assignment_ws/build/CMP9767M/uol_cmp9767m_base && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ActorCollisionsPlugin.dir/src/ActorCollisionsPlugin.cc.o -c /home/bonjour/NW_assignment_ws/src/CMP9767M/uol_cmp9767m_base/src/ActorCollisionsPlugin.cc

CMP9767M/uol_cmp9767m_base/CMakeFiles/ActorCollisionsPlugin.dir/src/ActorCollisionsPlugin.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ActorCollisionsPlugin.dir/src/ActorCollisionsPlugin.cc.i"
	cd /home/bonjour/NW_assignment_ws/build/CMP9767M/uol_cmp9767m_base && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bonjour/NW_assignment_ws/src/CMP9767M/uol_cmp9767m_base/src/ActorCollisionsPlugin.cc > CMakeFiles/ActorCollisionsPlugin.dir/src/ActorCollisionsPlugin.cc.i

CMP9767M/uol_cmp9767m_base/CMakeFiles/ActorCollisionsPlugin.dir/src/ActorCollisionsPlugin.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ActorCollisionsPlugin.dir/src/ActorCollisionsPlugin.cc.s"
	cd /home/bonjour/NW_assignment_ws/build/CMP9767M/uol_cmp9767m_base && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bonjour/NW_assignment_ws/src/CMP9767M/uol_cmp9767m_base/src/ActorCollisionsPlugin.cc -o CMakeFiles/ActorCollisionsPlugin.dir/src/ActorCollisionsPlugin.cc.s

CMP9767M/uol_cmp9767m_base/CMakeFiles/ActorCollisionsPlugin.dir/src/ActorCollisionsPlugin.cc.o.requires:

.PHONY : CMP9767M/uol_cmp9767m_base/CMakeFiles/ActorCollisionsPlugin.dir/src/ActorCollisionsPlugin.cc.o.requires

CMP9767M/uol_cmp9767m_base/CMakeFiles/ActorCollisionsPlugin.dir/src/ActorCollisionsPlugin.cc.o.provides: CMP9767M/uol_cmp9767m_base/CMakeFiles/ActorCollisionsPlugin.dir/src/ActorCollisionsPlugin.cc.o.requires
	$(MAKE) -f CMP9767M/uol_cmp9767m_base/CMakeFiles/ActorCollisionsPlugin.dir/build.make CMP9767M/uol_cmp9767m_base/CMakeFiles/ActorCollisionsPlugin.dir/src/ActorCollisionsPlugin.cc.o.provides.build
.PHONY : CMP9767M/uol_cmp9767m_base/CMakeFiles/ActorCollisionsPlugin.dir/src/ActorCollisionsPlugin.cc.o.provides

CMP9767M/uol_cmp9767m_base/CMakeFiles/ActorCollisionsPlugin.dir/src/ActorCollisionsPlugin.cc.o.provides.build: CMP9767M/uol_cmp9767m_base/CMakeFiles/ActorCollisionsPlugin.dir/src/ActorCollisionsPlugin.cc.o


# Object files for target ActorCollisionsPlugin
ActorCollisionsPlugin_OBJECTS = \
"CMakeFiles/ActorCollisionsPlugin.dir/src/ActorCollisionsPlugin.cc.o"

# External object files for target ActorCollisionsPlugin
ActorCollisionsPlugin_EXTERNAL_OBJECTS =

/home/bonjour/NW_assignment_ws/devel/lib/libActorCollisionsPlugin.so: CMP9767M/uol_cmp9767m_base/CMakeFiles/ActorCollisionsPlugin.dir/src/ActorCollisionsPlugin.cc.o
/home/bonjour/NW_assignment_ws/devel/lib/libActorCollisionsPlugin.so: CMP9767M/uol_cmp9767m_base/CMakeFiles/ActorCollisionsPlugin.dir/build.make
/home/bonjour/NW_assignment_ws/devel/lib/libActorCollisionsPlugin.so: CMP9767M/uol_cmp9767m_base/CMakeFiles/ActorCollisionsPlugin.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/bonjour/NW_assignment_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/bonjour/NW_assignment_ws/devel/lib/libActorCollisionsPlugin.so"
	cd /home/bonjour/NW_assignment_ws/build/CMP9767M/uol_cmp9767m_base && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ActorCollisionsPlugin.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMP9767M/uol_cmp9767m_base/CMakeFiles/ActorCollisionsPlugin.dir/build: /home/bonjour/NW_assignment_ws/devel/lib/libActorCollisionsPlugin.so

.PHONY : CMP9767M/uol_cmp9767m_base/CMakeFiles/ActorCollisionsPlugin.dir/build

CMP9767M/uol_cmp9767m_base/CMakeFiles/ActorCollisionsPlugin.dir/requires: CMP9767M/uol_cmp9767m_base/CMakeFiles/ActorCollisionsPlugin.dir/src/ActorCollisionsPlugin.cc.o.requires

.PHONY : CMP9767M/uol_cmp9767m_base/CMakeFiles/ActorCollisionsPlugin.dir/requires

CMP9767M/uol_cmp9767m_base/CMakeFiles/ActorCollisionsPlugin.dir/clean:
	cd /home/bonjour/NW_assignment_ws/build/CMP9767M/uol_cmp9767m_base && $(CMAKE_COMMAND) -P CMakeFiles/ActorCollisionsPlugin.dir/cmake_clean.cmake
.PHONY : CMP9767M/uol_cmp9767m_base/CMakeFiles/ActorCollisionsPlugin.dir/clean

CMP9767M/uol_cmp9767m_base/CMakeFiles/ActorCollisionsPlugin.dir/depend:
	cd /home/bonjour/NW_assignment_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/bonjour/NW_assignment_ws/src /home/bonjour/NW_assignment_ws/src/CMP9767M/uol_cmp9767m_base /home/bonjour/NW_assignment_ws/build /home/bonjour/NW_assignment_ws/build/CMP9767M/uol_cmp9767m_base /home/bonjour/NW_assignment_ws/build/CMP9767M/uol_cmp9767m_base/CMakeFiles/ActorCollisionsPlugin.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMP9767M/uol_cmp9767m_base/CMakeFiles/ActorCollisionsPlugin.dir/depend

