# MRDC 2020 Repository
This is the main repository for the MRDC 2020 team at the University of Illinois at Urbana-Champaign. This project is only designed to work on Linux. 

# Building
To build the entire project, simply clone the git repository, make a build directory with the name "cmake-build-\*" where "\*" is anything (technically any name will work, but this name will work better with the provided .gitignore), change into this directory, and then run a standard cmake+make build system. In-tree builds, builds with Cmake backends other than GNU make, and alternative build systems (e.g. autotools) are not supported, although other CMake backups will probably work.

    git clone <repo url>
    mkdir cmake-build-release # start this directory with "cmake-build" to ensure that it is properly .gitignored
    cd cmake-build-release
    cmake .. -DCMAKE_BUILD_TYPE=Release #set this to "Debug" to make a debug build, other options are listed in CMake's documentation
    make -j16 # change for your number of CPU cores

Partial builds (e.g. just the ROS nodes, or just the laptop code) beyond Arduino's separate setup (see its section) are not currently supported. This will eventually be supported via `-DMRDC_BUILD_TYPE=laptop|pi`, however.

# IDE Setup
Only CLion, by Jetbrains, is officially supported. Other IDEs that play nicely with CMake on Linux (e.g. Code::Blocks) will probably also work. To use this project in CLion, simply open the directory containing this README.md as a project, and CLion will automatically handle everything else (provided that correct libraries, such as ROS, are installed).

# Branches
When working on a node, branch off of master, make your changes, and then merge to master once a good chunk of changes is done. DO NOT DELETE ANY BRANCHES, EVER. Clutter is less bad than losing information.

# Project Structure 
This project extensively uses CMake's subdirectory setup to keep the various ROS nodes and other code separate. The following directories are CMake subprojects:
 - `ball/control-node` Ros node to manipulate servos, takes in information from `ball/image-classification` outputs to `serial-interface/rpi-node`. 
 - `ball/image-classification` Accesses USB webcam directly, outputs messages to ROS with the current color of the ball. Outputs clasification messages as frequently as possible (e.g. one for each processed frame) even if the ball color and/or ball do not change.
 - `drivetrain/control-node` Receives messages over xbee from `drivetrain/laptop` and emits messages that manipulate motors, etc. to `serial-interface\rpi-node`. Also receives battery level messages from the same node and forwards those to `drivetrain/laptop` over XBEE.
 - `drivetrain/laptop` Desktop application that reads from a controller over USB via SFML, and sends data to `drivetrain/control-node` over XBEE, receiving battery status information from the same pi node.
 - `gyroscope/control-node` Connects to IMU over USB, serial, or whatever on the pi and sends servo manipulation messages to `serial-interface/rpi-node`.
 - `serial-interface/arduino-code` The horrible, vile, evil, nasty monstrosity that is arduino's corruption of C++.
 - `serial-interface/rpi-node` The ros node that communicates with the evil and nasty monstrosity.
 
See the root level `CMakeLists.txt` to add additional nodes. Do not forget to document those nodes here.

# Arduino Code
The Arduino code is built separately than the rest, and uses the Arduino IDE. The above instructions for IDE setup and building do not apply to this code.