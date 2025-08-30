RTAB-MAP + QR-Code Autonomous Landing (Full README)
=================================================

This document describes how to use **Gazebo + PX4 SITL** to detect an AR/QR marker with a **downward-facing camera** for precise landing,
and how to enable **RTAB-MAP (RGB-D) mapping & localization** during **takeoff/flight**, feeding poses to PX4
via **/mavros/vision_pose/pose** to improve stability.

> You may also adapt this guide for real flights, but make sure to perform **camera calibration, time synchronization, parameter re-tuning, and safety fallbacks**.

-------------------------------------------------
1. Feature Overview
-------------------------------------------------
- Launch a quadrotor (iris) with a downward-facing camera in Gazebo.
- Use ar_track_alvar to detect the AR/QR marker on the landing pad and obtain the **relative pose** for alignment and convergence.
- Run RTAB-MAP (RGB-D) for online **mapping & localization**, using MAVROS local odometry (/mavros/local_position/odom) as odometry input;
  forward RTAB-MAP **localization poses** to PX4 via /mavros/vision_pose/pose to improve controller stability and robustness to temporary detection loss.
- Landing controller (state machine + PID): **WAITING → CHECKING → PREPARE → SEARCH → LANDING → LANDOVER → DONE**,
  with **lost-marker descent**, **robust landing criteria**, **final braking and optional auto disarm/termination**, etc.
- Supports both **aggregated launch** and **step-by-step launch**, with keyboard teleop and RViz for visualization.

-------------------------------------------------
2. Directory Layout & Key Files (example)
-------------------------------------------------
launch/
  - landing_px4.launch          # One-click entry (world, PX4+camera, ar_track, *optional* RTAB-MAP include, landing controller, keyboard, RViz)
  - camera_down_px4.launch      # PX4 SITL + downward depth camera + static TF (base_link → depth_camera_link)
  - landing.launch              # Landing controller node with PID/desired pose/safety parameters
  - rtabmap_rgbd.launch         # RTAB-MAP (RGB-D) mapping/localization (subscribes RGB-D + MAVROS odometry)

nodes/
  - odom_to_tf.py               # Optional: convert /mavros/local_position/odom into TF (odom → base_link_*)
  - rtabmap_to_mavros_pose.py   # Forward RTAB-MAP pose to /mavros/vision_pose/pose (with rate limiting)

src/
  - landing_quadrotor.cpp       # Core landing controller (state machine, robust landing checks, brake/disarm/termination)
  - landing_quadrotor.h         # Header (topics, services, parameters, PID structs)

(Note: Names above are indicative—use your actual package and paths.)

-------------------------------------------------
3. Dependencies (recommended)
-------------------------------------------------
- Ubuntu + ROS (Melodic / Noetic, etc.)
- Gazebo (matching your ROS)
- PX4-Autopilot (SITL) + MAVROS
- ar_track_alvar (or an equivalent AR/QR detector)
- rtabmap_ros + RTAB-Map
- RViz

-------------------------------------------------
4. Quick Start (Aggregated, Recommended)
-------------------------------------------------
1) Force-create a fresh RTAB-MAP database:
   roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start"

2) Launch the aggregated entry: 
   roslaunch <your_pkg> landing_px4.launch

3) Enter "rviz" in the terminal to manually enable it

   Ctrl + C to exit.
