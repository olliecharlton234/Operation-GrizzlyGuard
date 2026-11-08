# Operation-GrizzlyGuard
Group 15 - RS1 Operation GrizzlyGuard

**Scenario guide:** open [`scenario_guide.html`](scenario_guide.html) in a web browser for setup steps, the scenario commands, GUI controls and troubleshooting. Download it or open it from your local copy of this folder, because GitHub shows HTML files as code.

# GrizzlyGuard runbook

How to build and run every node, in which terminals and in what order. Ends with the next integration steps for decision making and perception.

Last updated 5 Nov 2026.

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
2. **Keep the GUI out of colcon.** It's built in Qt Creator:
   ```bash
   touch ~/RS1-Workspace/Operation-GrizzlyGuard/grizzlyguard_gui/COLCON_IGNORE
   ```
3. **ONNX Runtime**, needed by `grizzlyguard_perception`:
   ```bash
   cd ~/RS1-Workspace/Operation-GrizzlyGuard
   bash grizzlyguard_perception/scripts/install_onnxruntime.sh
   ```
   To build without perception for now, add `--packages-skip grizzlyguard_perception` to every `colcon build`.
4. **Every new Python node needs to be executable**, or `ros2 run` says "No executable found":
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

## 3. Run order

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

## 4. Node reference

| Node | Package / executable | Subscribes | Publishes |
|---|---|---|---|
| altitude_controller | grizzlyguard_navigation / `altitude_controller` | `/parrot1/cmd_vel` (Nav2), `/parrot1/target_altitude`, `/parrot1/odometry`, `/manual_mode` | `/parrot1/cmd_vel_gz` → Gazebo, `/parrot1/altitude_reached` |
| world_pose_publisher | grizzlyguard_navigation / `world_pose_publisher.py` | `/parrot1/odometry` | `/parrot1/world_pose` (PoseStamped, frame `world`) |
| patrol_node | grizzlyguard_navigation / `patrol_node.py` | `/manual_mode`, `/drone/takeoff`, `/drone/land`, `/drone/return_to_home`, `/drone/emergency_stop`, `/parrot1/altitude_reached` | Nav2 goals, `/parrot1/target_altitude`, `/parrot1/current_waypoint`, `/drone/status` |
| perimeter_status_node | grizzlyguard_decision / `perimeter_status_node.py` | `/bears/detections`, `/bears/lost` | `/perimeter/status` (UInt8 0–3), `/perimeter/alert` (String) |
| GUI (`bear_drone_gui_node`) | grizzlyguard_gui (Qt Creator) | camera, `/parrot1/world_pose`, `/bears/*`, `/perimeter/*`, `/deterrent/status`, `/drone/status` | `/cmd_vel` (manual), `/manual_mode`, `/drone/*` commands |

Command priority in patrol_node: **EMERGENCY > MANUAL > LAND > RETURN_HOME > TAKEOFF**.

Manual mode:
- The GUI owns `/cmd_vel`, and autonomy goes quiet.
- In manual mode, Takeoff/Land are flown by the GUI using the z from `/parrot1/world_pose`.
- When you leave manual mode, it resumes the patrol if the last button pressed was Takeoff, or stays landed if it was Land.

## 5. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `Package '…' not found` | Terminal not sourced, or a package skipped in the last build. `source install/setup.bash`; check `ros2 pkg prefix`. |
| `No executable found` | Python file not `chmod +x`, empty (0 bytes), or missing from `install(PROGRAMS …)` in CMakeLists. |
| One package fails, others "Aborted" | Fix the failing one or skip it: `--packages-skip <pkg>`. |
| `WAITING FOR NAV2` forever | Gazebo paused, Nav2 not started (`nav2:=true`), or terminals with different `ROS_LOCALHOST_ONLY`. Check `ros2 lifecycle get /parrot1/bt_navigator` and `grep -iE "error|fail" ~/sim.log`. |
| Blue dot doesn't move / manual land "needs altitude" | T3 not running, or it can't read best-effort odometry (QoS). |
| Drone doesn't move under autonomy | T2 not running, or bridge yaml not changed to `cmd_vel_gz`. |
| AutoMoc / "cannot be overloaded" errors in GUI | Code pasted twice or into the wrong file. Replace whole files instead of pasting. |
| Seeing someone else's robot | Set `ROS_LOCALHOST_ONLY=1`. |

---

## 6. Next integration steps: decision making and perception

**What works now:** patrol, altitude, the manual and autonomous hand-off, GUI commands, the blue dot, status display, and the traffic light and breach popup (when something publishes bears).

**What's missing:** nothing publishes real bear positions, there's no deterrent output, and the drone does not respond to bears.

The target data flow:
```
camera ─▶ perception (YOLO) ─▶ /parrot1/bear_detections (Detection2DArray, pixels)
                                        │
                              bear localiser + tracker
                                        │
                                        ▼
                         /bears/detections (MarkerArray, world frame), /bears/lost
                            │                         │
                            ▼                         ▼
               perimeter_status_node            response node (decision)
               /perimeter/status, /alert        /deterrent/status, /decision/target
                            │                         │
                            ▼                         ▼
                           GUI                patrol_node INVESTIGATE mode
```

### Step 0 – Agree on what "threat" means (decision team, do first)
`threat_zones.py` measures **bear → border centre (0, 0)**, but the comments in `patrol_routes.yaml` measure **drone → nearest bear**. These give different answers. Also, `bear_spawn` (4.5, 3.0) is 5.4 m from (0, 0), so it is **permanently EMERGENCY** with the current border.
- **Decide:** the traffic light shows bear → town distance (`border_x/y` = the town's real coordinates), while drone → bear distance only decides how close the drone flies.
- **Done when:** the border coordinates are written in `threat_zones.py` and passed to perimeter_status_node.

### Step 1 – Ground-truth bear publisher (decision, unblocks everything for the demo)
Publish the bears' **true** Gazebo poses on `/bears/detections` so the GUI, traffic light and response can be finished before perception is ready.
- Bridge Gazebo's model poses, e.g. `/world/yellowstone/pose/info` → `tf2_msgs/TFMessage` via `ros_gz_bridge`. Keep entries whose name starts with `bear_` and publish one `Marker` per bear (`ns: bear`, `id` stable per name, `frame_id: world`, `action: ADD`) at 2–5 Hz.
- Add a parameter `max_range` so only bears within camera range of the drone (`/parrot1/world_pose`) are "detected". That makes it behave like perception.
- **Done when:** red markers sit on the bears in the GUI and `bear_walk` moves; the traffic light changes as bears move.

### Step 2 – Perception output (perception team)
- Confirm the executable and topic: `ros2 pkg executables grizzlyguard_perception`, then run it and check `ros2 topic list`.
- Publish `vision_msgs/Detection2DArray` on `/parrot1/bear_detections`, with header stamp = the image stamp and `frame_id` = the camera frame.
- **Done when:** `ros2 topic echo /parrot1/bear_detections` shows boxes when a bear is in view, and nothing when none is.

### Step 3 – Pixel → world localisation (perception + decision)
`bear_perception_bridge.py` currently puts pixel coordinates into TF as metres, and its fallback places the bear relative to world (0, 0). Replace that with a ground-plane projection:
- Inputs: box bottom-centre pixel, camera intrinsics (`/parrot1/camera/camera_info` or the FOV from the URDF), camera pitch (`camera_joint` in `parrot.urdf.xacro`), and drone pose + altitude from `/parrot1/world_pose`.
- Cast a ray from the camera through the pixel and intersect it with the ground (z ≈ 0, see the note below). The intersection is the bear's world x/y.
- Use real frame names (`parrot1_*`, `world`), not `camera_link` / `map`.
- **Done when:** with a static bear in view, the estimated position is within ~2 m of the ground-truth pose from Step 1.

### Step 4 – Tracker (perception + decision)
- Associate detections to tracks by nearest world position (gate ~3 m) and give each a stable ID.
- Publish tracks as `/bears/detections` (MarkerArray, ADD). When a track isn't seen for N seconds (e.g. 5 s), publish `DELETE` for it and its ID on `/bears/lost`.
- Then swap the Step 1 ground-truth publisher for this one. Nothing downstream changes.
- **Done when:** a bear leaving view is removed from the GUI after the timeout, and IDs don't flicker.

### Step 5 – Deterrent output (decision)
- Publish `/deterrent/status` (`std_msgs/Bool`): `true` while a deterrent is active.
- Use `decision_logic.THREAT_DECISION_MAP`: YELLOW = sound, RED = spray + sound, deployed only when the drone is within 5–10 m of that bear.
- Optional: also publish `/deterrent/mode` (String: `NONE` / `SOUND` / `SPRAY_SOUND`) for a richer GUI label later.
- **Done when:** the GUI lamp turns orange during `red_zone` / `zone_approach` runs and turns off in GREEN.

### Step 6 – Bear response movement (decision + navigation)
`nav_follow_bridge.py` sends its own Nav2 goals. That cancels the patrol and **ignores manual mode and emergency stop**. Don't run it alongside patrol_node.
- Instead, the decision node publishes `/decision/target` (PoseStamped, world frame) with the stand-off point 5–10 m from the bear, and an empty/cleared target when the response ends.
- patrol_node gets an `INVESTIGATE` mode. It converts world → `parrot1_map` (subtract `spawn`) and goes to the target. On clear, it resumes the patrol from where it left off.
- The existing priorities stay: EMERGENCY and MANUAL interrupt INVESTIGATE.
- **Done when:** `zone_approach` (or the Step 1 publisher with `bear_walk`) makes the drone leave the route, hold near the bear with the deterrent on, then rejoin the route. `/drone/status` shows `INVESTIGATE …`.

### Step 7 – Package clean-up (decision)
- `CMakeLists.txt`: install **every** imported module (`threat_zones.py`, `decision_logic.py`, `decision_controller.py`, `bear_follow_controller.py`, `bear_perception_bridge.py`) next to the nodes, and `chmod +x` the node files.
- `package.xml`: add `vision_msgs`, `visualization_msgs`, `tf2_ros`, `tf2_geometry_msgs`.
- `classify_world_position` raises `ValueError` outside x −75..75 / y −40..235. Catch it, or use `classify_threat(distance_to_border_m(...))` as perimeter_status_node does.
- `Controller.cpp/.h` is an empty stub that duplicates the Python decision code. Delete it or decide which implementation is kept.

### Step 8 – One launch file (anyone, last)
Extend `navigation.launch.py`, or add `grizzlyguard_bringup/launch/grizzlyguard.launch.py`, to start T2–T5 plus the decision and perception nodes, with a short `TimerAction` delay after the sim. The demo then becomes two terminals: sim and everything else. Plus the GUI.

### Notes for whoever picks these up
- **Coordinate frames:** the GUI map, bears and threat zones all use the **Gazebo world frame**. Nav2 goals use `parrot1_map` (= world − spawn). Convert only at the Nav2 boundary.
- **Odometry z** is height above world zero, not above the terrain. Check the terrain height at the bears before using z ≈ 0 for ground-plane projection.
- **Topic contract:** the GUI listens on `/bears/...`, not `/bear/...` as in the requirements PDF. The code is what counts.
- **QoS:** subscribe to Gazebo-bridged topics (odometry, camera) with `qos_profile_sensor_data` / `SensorDataQoS`.
