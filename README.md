# Operation-GrizzlyGuard
Group 15 - RS1 Operation GrizzlyGuard

**Scenario guide:** open [`scenario_guide.html`](scenario_guide.html) in a web browser for setup steps, the scenario commands, GUI controls and troubleshooting. Download it or open it from your local copy of this folder, because GitHub shows HTML files as code.

# GrizzlyGuard runbook

How to set up, build and run GrizzlyGuard: a Parrot drone in a simulated Yellowstone forest that patrols, spots bears with YOLO, works out each bear's threat zone, and responds with deterrents, following and GUI alerts.

The quickest way to run it is one command per scenario (section 3). Section 4 shows how to start each part by hand, for debugging.

Last updated 8 Nov 2026.

---

## 1. One-time setup (per laptop)

**One workspace only:** `~/RS1-Workspace/Operation-GrizzlyGuard`. Never build anywhere else. A second `build/` or `install/` folder causes "Package not found" and "No executable found" errors.

1. **`~/.bashrc`**: source ROS only, no other workspaces:
   ```bash
   source /opt/ros/humble/setup.bash
   # source ~/41068_ws/install/setup.bash        <- keep old workspaces commented out
   export ROS_LOCALHOST_ONLY=1                  # only if GUI + sim run on the same laptop
   ```
   After editing, close **all** terminals and open new ones.
2. **The GUI isn't built by colcon.** `grizzlyguard_gui/COLCON_IGNORE` is already in the repo. Build the GUI in Qt Creator, or with cmake (section 3).
3. **ONNX Runtime**, needed by `grizzlyguard_perception`:
   ```bash
   cd ~/RS1-Workspace/Operation-GrizzlyGuard
   bash grizzlyguard_perception/scripts/install_onnxruntime.sh
   ```
4. **Packages and a fix** that the scenarios need:
   ```bash
   sudo apt update
   sudo apt install ros-humble-vision-msgs ros-humble-cv-bridge ros-humble-rqt-image-view
   sudo apt install --only-upgrade ros-humble-diagnostic-updater
   ```
   The last line fixes `ekf_node` crashing because `libdiagnostic_updater.so` is missing. Without it, the drone can't navigate.
5. **The two YOLO models.** They aren't in git, so put both in `grizzlyguard_perception/models/` before building:
   - `bear_custom.onnx`: the trained bear model. YOLO won't start without it. Get it from Oliver.
   - `yolo26n.onnx`: the standard model, used to spot people. Without it everything still runs, but hikers can set off false bear alarms. Get it from Mahak or Oliver, or make it with `grizzlyguard_perception/scripts/export_yolo.py`.
6. **WSL graphics.** If Gazebo crashes at start-up with an `Ogre` error, use software rendering by adding this to `~/.bashrc`:
   ```bash
   export LIBGL_ALWAYS_SOFTWARE=1
   ```
7. **Every new Python node needs to be executable**, or `ros2 run` says "No executable found":
   ```bash
   chmod +x grizzlyguard_navigation/scripts/*.py grizzlyguard_decision/*.py   # adjust to where the files are
   ```

## 2. Build

Rebuild only after changing C++, `CMakeLists.txt` or `package.xml`. With `--symlink-install`, Python edits take effect after restarting the node, without a rebuild.

```bash
cd ~/RS1-Workspace/Operation-GrizzlyGuard
source /opt/ros/humble/setup.bash
colcon build --symlink-install            # always use --symlink-install, never mix
source install/setup.bash
```

Check that every package comes from this workspace:
```bash
ros2 pkg prefix grizzlyguard_navigation   # -> .../Operation-GrizzlyGuard/install/grizzlyguard_navigation
ros2 pkg prefix grizzlyguard_decision
ros2 pkg prefix 41068_ignition_bringup
```

## 3. Run a scenario (one command)

`scenario.launch.py` starts everything for one test: Gazebo, the bears, SLAM, Nav2, the altitude controller, patrol, both YOLO detectors and the bear response. It starts the bears first, then the rest, and starts the patrol only once Nav2 is active.

```bash
cd ~/RS1-Workspace/Operation-GrizzlyGuard && source install/setup.bash
ros2 launch grizzlyguard_decision scenario.launch.py scenario:=yellow
```

| `scenario:=` | What's in the world | What should happen |
|---|---|---|
| `no_bear` | All bears hidden, hikers walking | Patrol only. Hikers are reported as "not a bear" |
| `green` | Bear 45–49 m from town | Bear reported, patrol continues |
| `yellow` | Bear 24–30 m from town | Sound, drone follows until the bear is in green, then patrols |
| `red` | Bear 11–14 m from town | Spray, then sound in yellow, follow until green, then patrol |
| `emergency` | Bear 6–9 m from town | Critical alert, drone follows, no deterrent |
| `not_a_bear` | All bears hidden, hikers only | Person reported, patrol continues |
| `none` (default) | All four bears walking normally | Normal patrol with whatever it finds |

E-stop, return home and manual control run on top of any scenario, from the GUI. Use `scenario:=no_bear gui:=true` to try them.

Other options: `gui:=true` opens the GUI (build it first, see below), `viewer:=false` hides the YOLO camera viewer, `rviz:=true` opens RViz, `route:=long_route` picks another patrol route, and `start_delay:=15.0` sets how long the drone gets to spawn first.

**GUI with `gui:=true`:** the launch file runs the GUI from `build/grizzlyguard_gui/`, so build it there first, and again whenever the GUI code changes:
```bash
source /opt/ros/humble/setup.bash
cmake -S grizzlyguard_gui -B build/grizzlyguard_gui
cmake --build build/grizzlyguard_gui -j4
```

To watch only the decisions, run this in a second terminal:
```bash
ros2 topic echo /perimeter/alert --field data
```

**To stop:** press Ctrl-C, and wait for the Gazebo window to close before launching again.

## 4. Run each part by hand (debugging)

**Every terminal starts with:**
```bash
cd ~/RS1-Workspace/Operation-GrizzlyGuard && source install/setup.bash
```

| # | What | Start after | Needed for |
|---|---|---|---|
| T1 | Simulation + SLAM + Nav2 | — | everything |
| T2 | altitude_controller | Gazebo is playing | any autonomous movement |
| T3 | world_pose_publisher | T1 | blue dot on the map, **manual takeoff/land** |
| T4 | patrol_node | Nav2 active (T1) + T2 | autonomy, GUI Takeoff/Land/RTH/E-stop |
| T5 | perimeter_status_node | anytime | traffic light, breach popup |
| T6 | GUI (Qt Creator) | T1 | operator |
| T7 | *(test only)* fake bear | T5/T6 | testing without perception |

This starts the drone, patrol and GUI only. For the bears, YOLO and the bear response, use the scenario launch in section 3. Don't run `perimeter_status_node` (T5) together with `bear_response_node`, because both drive the traffic light.

### T1 – Simulation
```bash
ros2 launch 41068_ignition_bringup 41068_ignition_parrot.launch.py slam:=true nav2:=true rviz:=false world:=yellowstone 2>&1 | tee ~/sim.log
```
Press **play** in Gazebo. Wait for Nav2 to come up (30–60 s on WSL):
```bash
ros2 lifecycle get /parrot1/bt_navigator     # want: active [3]
```
Use `rviz:=true` to show the planned path. Manual mode responds worse with RViz open.

### T2 – Altitude controller
```bash
ros2 run grizzlyguard_navigation altitude_controller --ros-args -r __ns:=/parrot1 \
    -r cmd_vel_nav:=cmd_vel -r cmd_vel:=cmd_vel_gz -p min_altitude:=0.2
```
OK when it prints `Odometry received, current z = 2.00 m`. Without this node, Nav2 cannot move the drone.

### T3 – World pose (blue dot)
```bash
ros2 run grizzlyguard_navigation world_pose_publisher.py --ros-args -p spawn_x:=0.0 -p spawn_y:=0.0
ros2 topic echo /parrot1/world_pose --once   # check: prints a pose
```
`spawn_x/y` must match the parrot's `add_robot(...)` in the launch file. world_pose_publisher must subscribe to odometry with `qos_profile_sensor_data`, or it receives nothing.

### T4 – Patrol
```bash
ros2 param get /parrot1/global_costmap/global_costmap origin_y    # should print -50.0
ros2 run grizzlyguard_navigation patrol_node.py --ros-args -p route:=short_route -p use_sim_time:=true
```
Routes are defined in `patrol_routes.yaml`: `short_route`, `long_route`, `green_zone`, `yellow_zone`, `red_zone`, `emergency_zone`, `zone_approach`. Add `-p cruise_altitude:=4.0` for a different height (keep it below the ~5 m canopy). If you change it, also change `kManualTakeoffAltitude` in the GUI's `simulationworker.cpp`.

Stuck on `WAITING FOR NAV2` → see Troubleshooting.

### T5 – Perimeter status (traffic light)
```bash
ros2 run grizzlyguard_decision perimeter_status_node.py
```
OK when it prints `Border centre (0.0, 0.0); publishing /perimeter/status at 2.0 Hz`. To move the border: `-p border_x:=… -p border_y:=…`.

### T6 – GUI
```bash
qtcreator grizzlyguard_gui/CMakeLists.txt
```
Qt Creator must be opened **from a sourced terminal**. First time, or after changing headers: Build → Run CMake, then Ctrl+B, then Run.

### T7 – Fake bear (testing only)
```bash
ros2 topic pub -r 2 /bears/detections visualization_msgs/msg/MarkerArray \
  "{markers: [{header: {frame_id: world}, ns: bear, id: 1, action: 0, pose: {position: {x: 12.0, y: 0.0}}}]}"
```
`x: 60` green · `30` yellow · `12` red · `5` red + breach popup. Ctrl+C → the bear is dropped after 5 s.

Other quick tests:
```bash
ros2 topic pub --once /deterrent/status std_msgs/msg/Bool "{data: true}"     # deterrent lamp
ros2 topic pub --once /drone/takeoff std_msgs/msg/Empty "{}"                 # takeoff without GUI
ros2 topic echo /drone/status                                                # what the drone is doing
```

### Emergency stop by hand
If a node dies mid-climb, Gazebo keeps applying the last velocity. To stop the drone:
```bash
ros2 topic pub --once /parrot1/cmd_vel_gz geometry_msgs/msg/Twist "{}"
```

## 5. Node reference

| Node | Package / executable | Subscribes | Publishes |
|---|---|---|---|
| altitude_controller | grizzlyguard_navigation / `altitude_controller` | `/parrot1/cmd_vel` (Nav2), `/parrot1/target_altitude`, `/parrot1/odometry`, `/manual_mode` | `/parrot1/cmd_vel_gz` → Gazebo, `/parrot1/altitude_reached` |
| world_pose_publisher | grizzlyguard_navigation / `world_pose_publisher.py` | `/parrot1/odometry` | `/parrot1/world_pose` (PoseStamped, frame `world`) |
| patrol_node | grizzlyguard_navigation / `patrol_node.py` | `/manual_mode`, `/drone/takeoff`, `/drone/land`, `/drone/return_to_home`, `/drone/emergency_stop`, `/parrot1/altitude_reached` | Nav2 goals, `/parrot1/target_altitude`, `/parrot1/current_waypoint`, `/drone/status` |
| bear_yolo_node | grizzlyguard_perception / `bear_yolo_node` | `/parrot1/camera/image` | `/parrot1/bear_detections`, `/parrot1/bear_detected`, `/parrot1/bear_detections/image` |
| person_yolo_node | same program, run with `yolo26n.onnx` and `target_class: person` (set up in `scenario.launch.py`) | `/parrot1/camera/image` | `/parrot1/nonbear_detections` |
| bear_response_node | grizzlyguard_decision / `bear_response_node.py` | `/parrot1/bear_detections`, `/parrot1/nonbear_detections`, `/parrot1/odometry`, `/parrot1/scan`, `/drone/status`, `/drone/emergency_stop`, `/drone/return_to_home` | `/bears/detections`, `/perimeter/status`, `/perimeter/alert`, `/deterrent/status`, `/bear_deterrent`, `/drone/follow`, `/parrot1/cmd_vel`, `/perception/not_bear` |
| bear_gait_demo | 41068_ignition_bringup / `bear_gait_demo.py` (with `pose_bridge`) | `/bear_deterrent` | Bear poses in Gazebo |
| perimeter_status_node | grizzlyguard_decision / `perimeter_status_node.py` | `/bears/detections`, `/bears/lost` | `/perimeter/status` (UInt8 0–3), `/perimeter/alert` (String) |
| GUI (`bear_drone_gui_node`) | grizzlyguard_gui (Qt Creator) | camera, `/parrot1/world_pose`, `/bears/*`, `/perimeter/*`, `/deterrent/status`, `/drone/status` | `/cmd_vel` (manual), `/manual_mode`, `/drone/*` commands |

Command priority in patrol_node: **EMERGENCY > MANUAL > LAND > RETURN_HOME > TAKEOFF**.

Manual mode:
- The GUI owns `/cmd_vel`, and autonomy goes quiet.
- In manual mode, Takeoff/Land are flown by the GUI using the z from `/parrot1/world_pose`.
- When you leave manual mode, it resumes the patrol if the last button pressed was Takeoff, or stays landed if it was Land.

## 6. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `Package '…' not found` | Terminal not sourced, or a package skipped in the last build. `source install/setup.bash`; check `ros2 pkg prefix`. |
| `No executable found` | Python file not `chmod +x`, empty (0 bytes), or missing from `install(PROGRAMS …)` in CMakeLists. |
| One package fails, others "Aborted" | Fix the failing one or skip it: `--packages-skip <pkg>`. |
| `WAITING FOR NAV2` forever | Gazebo paused, Nav2 not started (`nav2:=true`), or terminals with different `ROS_LOCALHOST_ONLY`. Check `ros2 lifecycle get /parrot1/bt_navigator` and `grep -iE "error|fail" ~/sim.log`. Nav2 also sometimes freezes at start-up (about 1 launch in 7): press Ctrl-C and launch again. |
| Blue dot doesn't move / manual land "needs altitude" | T3 not running, or it can't read best-effort odometry (QoS). |
| Drone doesn't move under autonomy | T2 not running, or bridge yaml not changed to `cmd_vel_gz`. |
| AutoMoc / "cannot be overloaded" errors in GUI | Code pasted twice or into the wrong file. Replace whole files instead of pasting. |
| Seeing someone else's robot | Set `ROS_LOCALHOST_ONLY=1`. |
| Gazebo crashes at start-up with an `Ogre` / `copyTo` error | WSL graphics problem. Set `export LIBGL_ALWAYS_SOFTWARE=1` (setup step 6). |
| YOLO node exits with "model file not found" | Put `bear_custom.onnx` and `yolo26n.onnx` in `grizzlyguard_perception/models/`, then rebuild. |
| Bears in the wrong place, or the patrol acts strangely after a restart | An old Gazebo may still be running. Close every Gazebo window, or run `pkill -f "ign gazebo"`, then launch again. |
| Camera very slow with the GUI open | Expected on laptops: it can drop below 1 frame per second. Run without `gui:=true` when you don't need the GUI. |

---

## 7. Known limitations

- **Return home** stops about 10 m from the start, at the same spot every run. The cause isn't confirmed.
- **People mistaken for bears:** the person check fixes most cases, but a hiker still caused a false bear response in 2 of 24 recorded runs.
- **Bear positions** assume flat ground and are only trusted within 15 m of the drone, so a bear is only judged when the patrol passes close to it.
- **One bear at a time:** every walking bear hears the deterrent, because YOLO can't tell bears apart.
- **Following** stops at obstacles instead of going around them.
- `grizzlyguard_decision/test_threat_logic.py` doesn't run, because it imports files that were removed.
