## OPTION A
sudo apt update
sudo apt install -y qtcreator qt6-base-dev qt6-base-dev-tools build-essential cmake ninja-build pkg-config libsdl2-dev

cd ~/RS1-Workspace/Operation-GrizzlyGuard
source /opt/ros/humble/setup.bash
source install/setup.bash
qtcreator grizzlyguard_gui/CMakeLists.txt

## OPTION B _ NO TERMINAL 

cd ~/RS1-Workspace/Operation-GrizzlyGuard
source /opt/ros/humble/setup.bash
source install/setup.bash

cmake -S grizzlyguard_gui -B grizzlyguard_gui/build -G Ninja
cmake --build grizzlyguard_gui/build

./grizzlyguard_gui/build/BearDroneGUI      # start Gazebo first

**NOTE YOU DO NOT NEED TO RUN RVIS** 
most information is showed in the gui simulation anyways