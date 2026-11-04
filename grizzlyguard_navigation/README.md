# COPY PASTE THESE COMMANDS 
## TERMINAL 1 - set up simulation environment
cd ~/RS1-Workspace/Operation-GrizzlyGuard
colcon build --symlink-install --packages-select 41068_ignition_bringup
source install/setup.bash
ros2 launch 41068_ignition_bringup 41068_ignition_parrot.launch.py slam:=true nav2:=true rviz:=true world:=yellowstone 2>&1 | tee ~/sim.log

## Terminal 2 - run altitude
cd ~/RS1-Workspace/Operation-GrizzlyGuard && source install/setup.bash
ros2 run grizzlyguard_navigation altitude_controller --ros-args -r __ns:=/parrot1 -r cmd_vel_nav:=cmd_vel -r cmd_vel:=cmd_vel_gz -p min_altitude:=0.2


## TERMINAL 3 - run patrol_node
cd ~/RS1-Workspace/Operation-GrizzlyGuard && source install/setup.bash
ros2 param get /parrot1/global_costmap/global_costmap origin_y     # should print -50.0
ros2 run grizzlyguard_navigation patrol_node.py --ros-args -p route:=short_route -p use_sim_time:=true

## Terminal 4 ** optional **, to see where the current progress on patrol path 
run this in a terminal, it will show what number waypoint the drone is on

cd ~/RS1-Workspace/Operation-GrizzlyGuard && source install/setup.bash
ros2 topic echo /parrot1/follow_waypoints/_action/feedback --field feedback.current_waypoint

# IGNORE BELOW 



## build the environment
source /opt/ros/humble/setup.bash
source ~/RS1-Workspace/Operation-GrizzlyGuard/install/setup.bash
export ROS_LOCALHOST_ONLY=1



## you will need two terminals to run this code, the first terminal runs the simulation which is 

'"ros2 launch 41068_ignition_bringup 41068_ignition_parrot.launch.py slam:=true nav2:=true rviz:=true world:=yellowstone"'

## use this line to run the patrol node

source ~/RS1-Workspace/Operation-GrizzlyGuard/install/setup.bash
source ~/RS1-Workspace/install/setup.bash
ros2 run grizzlyguard_navigation patrol_node.py --ros-args -p route:=long_route

or run these once 
echo 'source ~/RS1-Workspace/Operation-GrizzlyGuard/install/setup.bash' >> ~/.bashrc
echo 'source ~/RS1-Workspace/install/setup.bash' >> ~/.bashrc

and then 
ros2 run grizzlyguard_navigation patrol_node.py --ros-args -p route:=long_route


**to run each route**
set route to one of these options: 
* long_route
* medium_route
* short_route 
* green_zone 
* yellow_zone 
* orange_zone
* red_zone 
* emergancy_zone 
* zone_approach

