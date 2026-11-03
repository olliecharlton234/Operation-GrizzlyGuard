# build the environment
source /opt/ros/humble/setup.bash
source ~/RS1-Workspace/Operation-GrizzlyGuard/install/setup.bash
export ROS_LOCALHOST_ONLY=1



# you will need two terminals to run this code, the first terminal runs the simulation which is 

'"ros2 launch 41068_ignition_bringup 41068_ignition_parrot.launch.py slam:=true nav2:=true rviz:=true world:=yellowstone"'

# use this line to run the patrol node

  cd ~/RS1-Workspace/Operation-GrizzlyGuard
  source install/setup.bash
  ros2 action list | grep follow_waypoints   # should show /parrot1/follow_waypoints
  ros2 run grizzlyguard_navigation patrol_node --ros-args -p use_sim_time:=true

  
ros2 run grizzlyguard_navigation patrol_node --ros-args -r __ns:=/parrot1 -p use_sim_time:=true