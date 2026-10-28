# M13 — Continuous Autonomous 3D Exploration

**Status:** Demonstrated in simulation
**Platform:** ROS 2 Jazzy, PX4 SITL, Gazebo Harmonic
**Localization:** Point-LIO
**Mapping:** Live OctoMap
**Planning:** Custom frontier selection + custom live 3D A*
**Execution:** Custom PX4 Offboard exploration supervisor

## 1. Objective

M13 integrates the localization, mapping, planning, and flight-control
work from the earlier milestones into a continuous GPS-denied
autonomous exploration system.

The system is designed to:

- localize using Point-LIO,
- fuse external-vision localization into PX4 EKF2,
- grow a 3D OctoMap online,
- detect unexplored frontiers,
- select safe frontier goals,
- generate collision-aware 3D paths,
- execute those paths through PX4 Offboard control,
- detect local path blockage,
- hold and trigger immediate replanning when necessary,
- continue exploring,
- and land safely if exploration or recovery becomes impossible.
The demonstrated system remains simulation-tested and is still being
tuned for smoother exploration and fewer unnecessary replans.

## 2. Stable Autonomy Architecture

The current stable architecture is:

    Gazebo LiDAR + IMU
             |
             v
          Point-LIO
             |
             v
    GPS-denied localization
             |
             v
    PX4 EKF2 external-vision fusion
             |
             v
      Live OctoMap mapping
             |
             v
    M13C frontier detection
    + safe goal selection
             |
             v
       M13D live 3D A*
             |
             v
       /planned_path
             |
             v
    M13G exploration supervisor
             |
             v
      PX4 Offboard setpoints
A lightweight safety layer operates in parallel:

    LiDAR / Point-LIO cloud
             |
             v
     Local path monitor
             |
             v
    Upcoming path blocked
             |
             v
          M13G HOLD
             |
             v
    Immediate M13D replan
             |
             v
       New valid path
             |
             v
      Continue exploration

This stack deliberately does not use the Nav2 controller.
The project retains a custom three-dimensional autonomy pipeline,
with PX4 providing the low-level flight control.
## 3. Core ROS 2 Topics

| Purpose | Topic |
|---|---|
| Point-LIO odometry | `/aft_mapped_to_init` |
| Point-LIO registered cloud | `/cloud_registered` |
| Body-frame registered cloud | `/cloud_registered_body` |
| Live OctoMap | `/octomap_binary` |
| Selected frontier goal | `/selected_frontier_goal` |
| Planned 3D path | `/planned_path` |
| PX4 external vision input | `/fmu/in/vehicle_visual_odometry` |
| Active executed path | `/m13g/active_path` |
| Local path blocked flag | `/local_path_blocked` |
| Immediate replan request | `/planner_replan_now` |

## 4. GPS-Denied Localization and PX4 Fusion
Point-LIO provides LiDAR-inertial localization.

The localization estimate is converted from the ROS / Point-LIO
coordinate convention into PX4 NED before being published as
external vision.

The position conversion is:

    ROS X -> PX4 Y
    ROS Y -> PX4 X
    ROS Z -> -PX4 Z

External-vision fusion was verified in PX4 using estimator status
diagnostics, including:

    cs_ev_pos: True
The external-vision bridge also rejects stale localization
measurements rather than forwarding delayed pose estimates to PX4.

## 5. Live 3D Mapping

OctoMap is generated online from:

    /cloud_registered_body

Validated mapping configuration:

| Parameter | Value |
|---|---:|
| Resolution | 0.10 m |
| Frame | `camera_init` |
| Sensor maximum range | 5.5 m |

This allows the occupancy map to expand while the drone moves into
previously unseen areas.

## 6. M13C — Live Frontier Detection and Safe Goal Selection

The custom node:

    frontier_detector_live

performs:

- live frontier extraction from OctoMap,
- frontier clustering,
- candidate filtering,
- obstacle-clearance checking,
- safe standoff generation,
- distance filtering,
- candidate scoring,
- candidate ranking,
- and final safe frontier-goal selection.
Current key parameters:

| Parameter | Value |
|---|---:|
| Horizontal clearance | 0.50 m |
| Vertical clearance | 0.30 m |
| Frontier altitude | 1.55 m |
| Standoff | 1.00–1.60 m |
| Minimum goal distance | 1.00 m |
| Maximum goal distance | 10.0 m |

Recent testing produced examples with:

- 17 retained frontier clusters,
- 15 safe candidates,
- ranked frontier scores,
- and automatic selection of the highest-ranked safe goal.

The selected planner goal is published on:

    /selected_frontier_goal

## 7. M13D — Live 3D A* Planning

The custom node:

    astar_3d_live

plans directly through the current 3D OctoMap.

Features include:

- occupied-space rejection,
- handling of unknown space,
- horizontal and vertical obstacle clearance,
- corner-cut protection,
- path validation,
- use of the current Point-LIO pose as the planning start,
- safe-start snapping near frontier boundaries,
- periodic replanning,
- and immediate event-triggered replanning.
Current global planner clearance:

| Parameter | Value |
|---|---:|
| Horizontal clearance | 0.50 m |
| Vertical clearance | 0.30 m |

Safe-start recovery currently allows approximately:

| Parameter | Value |
|---|---:|
| Horizontal search radius | 0.80 m |
| Vertical snap limit | 0.15 m |

The vertical snap limit prevents recovery paths from beginning
substantially above or below the drone's actual altitude.
Immediate replanning can be requested through:

    /planner_replan_now

Periodic replanning remains available as a fallback.

## 8. M13G — Continuous Exploration Supervisor

The custom node:

    m13g_continuous_explorer

coordinates flight execution and mission state.

Its state flow is approximately:

    WAIT_READY
        |
        v
    PRESTREAM
        |
        v
    WAIT_OFFBOARD
        |
        v
    WAIT_ARM
        |
        v
    TAKEOFF
        |
        v
    WAIT_FIRST_PATH
        |
        v
    FOLLOW_PATH
        |
        +----> HOLD / WAIT_REPLAN
        |              |
        |              v
        +-------- FOLLOW_PATH
        |
        v
    LANDING
        |
        v
    COMPLETE
The supervisor performs:

- automatic PX4 Offboard entry,
- automatic arming,
- autonomous bootstrap takeoff to approximately 1.55 m,
- waiting for a valid live 3D A* path,
- execution of 3D path waypoints,
- acceptance of newly generated paths,
- hold behavior when the local path is blocked,
- immediate replan requests,
- automatic landing when exploration or recovery becomes impossible,
- and automatic disarming after landing.

Current follower tolerances:

| Parameter | Value |
|---|---:|
| Horizontal waypoint tolerance | 0.15 m |
| Vertical waypoint tolerance | 0.15 m |
Path-start safety validation:

| Parameter | Value |
|---|---:|
| Maximum horizontal start error | 0.80 m |
| Maximum vertical start error | 0.20 m |

## 9. Point-LIO Map Frame to PX4 Local Frame Alignment

A significant integration issue was discovered during M13 testing.

Converting ENU axes into NED was not sufficient because the
Point-LIO `camera_init` origin and PX4 local-position origin were
not always identical after restart.

M13G was updated to calculate and apply the translational offset
between:

- Point-LIO `camera_init`,
- and PX4 local NED.

Example diagnostic:

    MAP->PX4 NED offset: [-0.02, 0.00, 0.07] m

Following this correction, recovery paths showed substantially
better vertical alignment.

Example:

    Start horizontal error: 0.596 m
    Start vertical error:   0.069 m
The 0.20 m vertical rejection threshold remains active as an
additional safety guard.

## 10. Local Obstacle Monitoring and Reactive Replanning

The custom node:

    local_path_monitor

provides a lightweight local safety layer for the path currently
being executed by M13G.

The first implementation was too sensitive and repeatedly triggered
on small numbers of raw LiDAR points.
This produced excessive:

- stopping,
- replanning,
- slow motion,
- and timeout-triggered landing.

The monitor was therefore tuned to operate as an emergency /
path-change detector rather than duplicating the full global
A* collision envelope.

Current settings:

| Parameter | Value |
|---|---:|
| Lookahead | 1.0 m |
| Horizontal corridor | 0.35 m |
| Vertical corridor | 0.20 m |
| Minimum obstacle hits | 8 points |
| Block confirmation | 4 scans |
| Clear confirmation | 2 scans |
| Point stride | 2 |

The design principle is:

    Global 3D A*
        = conservative collision-aware planning

    Local path monitor
        = fast active-path blockage detection

When blockage is confirmed:

    local_path_monitor
          |
          v
    /local_path_blocked
          |
          v
       M13G HOLD
          |
          v
    /planner_replan_now
          |
          v
       M13D replans
          |
          v
      Continue flight
## 11. Safe Failure Behavior

The exploration stack prefers a safe failure mode.

If M13G cannot obtain a valid initial path or recovery path within
its allowed timeout, it requests landing rather than continuing
blindly.

This safe fallback behavior has been observed successfully during
simulation testing.

## 12. Experimental Frontier Goal Manager

An experimental:

    frontier_goal_manager.py

was developed to test cycling through multiple frontier candidates.

It is not part of the current stable stack.
Testing revealed that the RViz visualization marker stream was not
a reliable planner-goal interface and could differ from the final
safe standoff goal chosen internally by M13C.

The current stable planner interface therefore remains:

    M13C
    /selected_frontier_goal
          |
          v
    M13D

A future improvement would publish ranked safe standoff goals through
a dedicated planning interface rather than reconstructing planner
goals from visualization markers.
## 13. Demonstrated Results

The integrated system has demonstrated:

- GPS-denied autonomous flight,
- Point-LIO localization,
- PX4 EKF2 external-vision fusion,
- live 3D LiDAR mapping,
- online OctoMap expansion,
- live frontier detection,
- frontier clustering,
- safe frontier-goal selection,
- custom live 3D A* planning,
- autonomous takeoff,
- autonomous path execution,
- repeated frontier exploration,
- continuous autonomous exploration,
- local obstacle detection,
- hold and event-driven replanning,
- obstacle avoidance / recovery,
- and autonomous landing and disarming when recovery fails safely.
The obstacle-avoidance behavior has been demonstrated successfully
in simulation but remains under tuning for smoother motion and fewer
unnecessary replans.

## 14. Demonstration Media

### Continuous Autonomous Exploration — Part 1

![Continuous autonomous exploration part 1](../media/m13/full_exploration_1.gif)

### Continuous Autonomous Exploration — Part 2

![Continuous autonomous exploration part 2](../media/m13/full_exploration_2.gif)

### Online 3D Mapping and Map Growth
![Online Point-LIO and OctoMap growth](../media/m13/map_growth.gif)

### Frontier Selection and Live 3D A* Replanning

![Frontier selection and live replanning](../media/m13/frontier_replanning.gif)

### Local Obstacle Avoidance and Recovery

![Obstacle avoidance and recovery](../media/m13/obstacle_avoidance.gif)

## 15. M13 Sub-Milestones

| Sub-milestone | Result |
|---|---|
| M13A — Live OctoMap | PASS |
| M13B — Live frontier detection | PASS |
| M13C — Clustering + safe frontier selection | PASS |
| M13D — Live 3D A* replanning | PASS |
| M13E — Autonomous execution of A* paths | PASS |
| M13F — Repeated frontier exploration | PASS |
| M13G — Continuous autonomous exploration | Demonstrated |

The local obstacle-monitoring and event-driven replanning layer is a
subsequent robustness improvement beyond the original M13G
continuous-exploration implementation.
## 16. Published Custom Nodes

The GitHub portfolio repository includes the stable custom nodes:

    ros2_ws/src/drone_3d_planner/src/frontier_detector_live.cpp
    ros2_ws/src/drone_3d_planner/src/astar_3d_live.cpp

    ros2_ws/src/drone_offboard_control/drone_offboard_control/
        m13g_continuous_explorer.py
        local_path_monitor.py
The portfolio copies were independently build-tested using ROS 2
Jazzy and colcon.

The following executables were successfully registered:

    drone_3d_planner frontier_detector_live
    drone_3d_planner astar_3d_live
    drone_offboard_control m13g_continuous_explorer
    drone_offboard_control local_path_monitor
## 17. Current Limitations

The current system should not be interpreted as:

- flawless dynamic obstacle avoidance,
- a production-ready exploration system,
- a Nav2-based controller,
- or complete hardware validation of the exploration stack.

Current development focuses on improving exploration smoothness,
reducing unnecessary replans, and increasing robustness in more
challenging environments.
