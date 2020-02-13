# MRDC 2020 Repository
This is the main repository for the MRDC 2020 team at the University of Illinois at Urbana-Champaign. This project is only designed to work on Linux. It consists of a catkin workspace. Each ROS package (which generally means one node) should be contained as a single subfolder of ros/src. Non-ROS packages should each have a top level subdirectory with their own build systems.

# Building
To build the entire ROS setup, simply clone the git repository, and then run Catkin. Make sure that your ROS installation is sourced (See ROS documentation for details.). To build the rest of the sub-projects (e.g. arduino, desktop program) refer to their specific README.md files or the standard practice for their platforms.

```
git clone <url> <folder>
cd <folder>
catkin_make
``` 

# Working With This Repository in CLion
Only CLion, by Jetbrains, is officially supported. Other IDEs that play nicely with CMake on Linux (e.g. Code::Blocks) will possibly also work. To work with this Catkin workspace in CLion, first launch CLion from a terminal that has ROS sourced (see ROS documentation for details), then open the repository root in CLion, and set the CMakeLists.txt to the one in the src directory. To work with another project in this repository (non-ROS) simply set the CMakeLists, cargo.toml, etc. to the file in CLion. 

# Branches
When working on a feature, branch off of master, make your changes, and then merge to master once a good chunk of changes is done. **DO NOT DELETE ANY BRANCHES, EVER.** Clutter is less bad than losing information.