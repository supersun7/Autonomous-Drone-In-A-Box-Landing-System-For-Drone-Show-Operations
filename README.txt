QR-Code Autonomous Landing (No RTAB-MAP Version)
=================================================

This README explains how to run a Gazebo + PX4 SITL setup to detect an AR/QR marker with a downward-facing camera and perform precise autonomous landing.
This version **does not include RTAB-MAP mapping**. It only covers the full loop of takeoff/hover/search/landing plus manual keyboard control.

-------------------------------------------------
1) Feature Overview
-------------------------------------------------
- Launch a quadrotor (iris) in Gazebo with a downward-facing camera.
- Use ar_track_alvar to detect the AR/QR marker on the landing pad and obtain relative pose.
- Close the loop with PID to align (x/y/z/yaw); once conditions are met, perform a short brake and switch to auto landing.
- A keyboard control script is provided to ARM / TAKEOFF / switch OFFBOARD / LAND / POSCTL / STABILIZED / MISSION, etc.
- Both one-click (aggregated) launch and step-by-step launch are available.

-------------------------------------------------
2) Directory & Key Files (example)
-------------------------------------------------
launch/
  ├─ landing_px4.launch           # One-click start: Gazebo + camera + ar_track + landing node + keyboard control + RViz
  ├─ camera_down_px4.launch       # Start PX4 SITL + downward depth camera + TF
  ├─ landing.launch               # Start landing controller node and load PID / desired pose parameters
  └─ mavros_posix_sitl.launch     # Standard MAVROS + PX4 SITL wrapper (included by other launch files)

worlds/
  ├─ landing_place.world          # Scene with a "landing pad" (with AR/QR texture)
  └─ city.world                   # Example city/model scene (with camera plugin & image topics)

src/
  ├─ landing_quadrotor.cpp        # Landing control logic (state machine + PID)
  └─ landing_quadrotor.h          # Header (topics/services/state machine/parameter structs, etc.)

scripts/
  └─ keyboard_control_px4.py      # Keyboard teleop + mode switching

(The above are reference names; use your actual package name and paths.)

-------------------------------------------------
3) Dependencies (recommended)
-------------------------------------------------
- Ubuntu + ROS (Melodic / Noetic, etc.)
- Gazebo (matching your ROS)
- PX4 SITL (PX4-Autopilot)
- MAVROS
- ar_track_alvar (or an equivalent AR/QR detection pipeline)
- RViz

Refer to the official docs of each project for installation and build.

-------------------------------------------------
4) Build
-------------------------------------------------
Place this project into your catkin workspace:
  ~/catkin_ws/src/<your_pkg>/
Then:
  $ cd ~/catkin_ws
  $ catkin_make
  $ source devel/setup.bash

-------------------------------------------------
5) Quick Start (one-click)
-------------------------------------------------
Recommended aggregated launch:
  $ roslaunch <your_pkg> landing_px4.launch

This launch will:
- Select the scene worlds/landing_place.world
- Start PX4 SITL + UAV with downward depth camera + TF
- Start ar_track_alvar (camera calibration/topics within its own config)
- Start the landing controller (load PID and desired pose params)
- Start the keyboard control script and RViz view

(If your package name differs, replace <your_pkg> accordingly.)

-------------------------------------------------
6) Step-by-Step Launch (optional)
-------------------------------------------------
If you need split debugging, open multiple terminals in this order:

1) Gazebo + PX4 + downward camera
   $ roslaunch <your_pkg> camera_down_px4.launch

2) ar_track_alvar detection (example)
   $ roslaunch ros_vision ar_track_camera.launch

3) Landing controller
   $ roslaunch px4_control landing.launch \
       search_alt_:=3 markers_id_:=4 \
       desire_pose_x:=0 desire_pose_y:=0 desire_pose_z:=0.2 desire_yaw_:=1.4 \
       PidXY_p:=1 PidXY_d:=0.1 PidXY_i:=0.02 \
       PidZ_p:=0.2 PidZ_d:=0.1 PidZ_i:=0.0 \
       PidYaw_p:=0.1 PidYaw_d:=0.0 PidYaw_i:=0.0

4) (Optional) Keyboard control & RViz
   $ rosrun simulation keyboard_control_px4.py
   $ rosrun rviz rviz -d $(rospack find ros_vision)/config/ar_track_camera.rviz

-------------------------------------------------
7) Recommended Workflow
-------------------------------------------------
1) After Gazebo starts, wait for PX4/MAVROS to connect properly.
2) Use the keyboard script for modes/throttle:
   - '0' ARM, '1' takeoff (AUTO.TAKEOFF), '2' OFFBOARD, '3' AUTO.LAND
   - '4' POSCTL, '5' STABILIZED, '6' AUTO.MISSION
   - 'i/k' throttle, 'e/d' pitch, 's/f' roll, 'j/l' yaw; 'g/h' decrease/increase control speed grade
3) Once in OFFBOARD, the landing node starts:
   - UAV climbs to search_alt_ (search altitude) and hovers to search for the marker
   - After detection, it enters alignment & descent; when convergence thresholds are met, it briefly brakes and switches to AUTO.LAND to finish

-------------------------------------------------
8) Key Parameters (configurable in landing.launch / landing_px4.launch)
-------------------------------------------------
- search_alt_        Search altitude (m), e.g., 3
- markers_id_        Target marker ID (integer)
- desire_pose_x/y/z  Desired relative pose (m). When desire_pose_z is small enough (e.g., 0.2–0.3), final landing is triggered
- desire_yaw_        Desired yaw (rad)
- PidXY_*            PID for x/y (p/d/i)
- PidZ_*             PID for z (p/d/i)
- PidYaw_*           PID for yaw (p/d/i)

Note: yaw=0 reference is defined as the orientation baseline when marker #0 is at the top-left of the camera image.

-------------------------------------------------
9) State Machine (landing controller)
-------------------------------------------------
WAITING     : Wait for OFFBOARD; no control if not entered
CHECKING    : Check position; if abnormal, switch to AUTO.LAND and wait again
PREPARE     : Climb to search_alt_ (search altitude)
SEARCH      : Keep searching if the marker is not detected; upon detection, go to LANDING
LANDING     : PID alignment for x/y/z/yaw; when height/centering/velocity meet thresholds, switch to FINAL_BRAKE
FINAL_BRAKE : Brake on the spot for ~1 s
LANDOVER    : Stop sending setpoints, switch to AUTO.LAND, end control loop
DONE        : Terminal state (node stops controlling)

-------------------------------------------------
10) Topic Interfaces (common)
-------------------------------------------------
- Sub: /ar_pose_marker (from ar_track_alvar; includes target ID and relative pose)
- Sub: /mavros/local_position/pose (local ENU pose)
- Sub: /mavros/state (PX4 mode and connection state)
- Srv: /mavros/set_mode (switch OFFBOARD / AUTO.LAND, etc.)
- (Internal) OffboardControl publishes desired position or body-frame velocity setpoints

Camera (Gazebo depth camera plugin) common topics:
- /camera/rgb/image_raw
- /camera/depth/image_raw
(Refer to the actual plugin config.)

-------------------------------------------------
11) Tuning & Tips
-------------------------------------------------
- Start with conservative output limits in PID to avoid “orbiting” or oscillations near the pad.
- If detection is lost, the controller gently commands a downward (negative z) velocity to re-acquire the marker; reduce this descent speed if needed.
- The final landing trigger uses triple criteria—altitude, lateral error, and lateral velocity—and requires short stability, improving reliability.
- If stuck in WAITING, check whether OFFBOARD is active and that time stamps / simulation clock are synchronized.

-------------------------------------------------
12) Gazebo Scenes & Customization
-------------------------------------------------
- The default landing_place.world contains a ~0.5 m × 0.5 m "landing pad". Replace its texture with your own AR/QR image if needed.
- You can switch world files via the world_path parameter in landing_px4.launch.
- For city/world or other scenes, ensure an AR/QR marker is visible in the camera view.

-------------------------------------------------
13) Known Limitations
-------------------------------------------------
- RTAB-MAP / SLAM is not integrated in this version; no mapping/localization enhancement during takeoff.
- Verified in simulation only; for real flights, re-tune according to your camera calibration, latency, and safety requirements.

-------------------------------------------------
14) Acknowledgments
-------------------------------------------------
- PX4 / MAVROS / Gazebo / ar_track_alvar open-source communities.

-------------------------------------------------
Appendix: Keyboard Quick Reference
-------------------------------------------------
Motion:
  e/d: pitch   s/f: roll   i/k: throttle   j/l: yaw
  g/h: decrease/increase control speed grade

Mode commands:
  0: ARM    1: TAKEOFF    2: OFFBOARD
  3: LAND   4: POSCTL     5: STABILIZED
  6: MISSION

Ctrl + C to exit.
