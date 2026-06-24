# openamrobot_bringup

Top-level launch composition for OpenAMRobot. Wires subsystems together; contains no
low-level nodes, models, or Nav2 parameters of its own (those live in their packages).

## `bringup.launch.py` — pick simulation or real

The single entry point. The **same** Nav2 stack and the **same** `nav2_params.yaml` run in
both cases; only the **data source** and the **clock** change.

```bash
ros2 launch openamrobot_bringup bringup.launch.py                       # real hardware (default)
ros2 launch openamrobot_bringup bringup.launch.py sim:=true use_rviz:=true   # Gazebo
ros2 launch openamrobot_bringup bringup.launch.py map:=/path/to/real_map.yaml
```

| `sim:=` | Data source | Window(s) | `use_sim_time` | Hardware |
|---|---|---|---|---|
| **false** (default) | `real_bringup.launch.py` (drivers + perception + EKF + TFs) | RViz only, if `use_rviz:=true` | `false` (real clock) | **required** + a real map |
| **true** | `openamrobot_gazebo` (Gazebo + gz_bridge + robot_state_publisher) | **Gazebo** + optional RViz | `true` (Gazebo clock) | none |

| Argument | Default | Meaning |
|---|---|---|
| `sim` | `false` | `true` = Gazebo simulation, `false` = real hardware |
| `use_rviz` | `false` | open RViz with the Nav2 view |
| `map` | `…/openamrobot_nav2/maps/my_map.yaml` | map for AMCL (pass your real map when `sim:=false`) |

### Sending a navigation goal (RViz "2D Goal Pose")

`navigation_launch.py` remaps `bt_navigator`'s goal input `goal_pose → goal_pose_nav` so the
**docking** node can gate `/goal_pose` (undock-before-navigate). So a goal published on
`/goal_pose` only reaches Nav2 if something forwards it to `/goal_pose_nav`:

- **sim** — the docking node (`openamrobot_docking`, run separately) forwards it.
- **real** — the docking pipeline is **not ported yet** (Gazebo-wired apriltag), so this launch
  starts a small `topic_tools relay /goal_pose → /goal_pose_nav` (real profile only). The
  standard RViz goal tool (publishes `/goal_pose`) therefore works in both profiles.

**Roadmap:** once the real AprilTag docking is ported (real camera → apriltag_ros →
`/detected_dock_pose`), replace the relay with the `dock_trigger` node so the real robot gets
the full undock-before-navigate gating **and** docking (this is "option B").

## `real_bringup.launch.py`

Brings up the **real-robot data sources** so the same Nav2 / SLAM / docking stack runs on
hardware as in simulation. Only the data source differs:

| Layer | Simulation | Real (this launch) |
|---|---|---|
| odom / scan / imu / camera | `openamrobot_gazebo` (Gazebo + `gz_bridge`) | `openamrobot_drivers` + `openamrobot_perception` + EKF |

It composes:
- `openamrobot_drivers/drivers.launch.py` — micro-ROS agent (Teensy) + RPLIDAR → `/scan`;
- `openamrobot_perception/scan_body_filter.launch.py` — `/scan` → `/scan_filtered`;
- `openamrobot_perception/camera.launch.py` — `camera_ros` (IMX708);
- `robot_localization` **EKF** (`config/ekf.yaml`) — wheels + IMU gyro-Z → `/odom` + TF
  `odom→base_link`;
- **measured static TFs** for this unit (`base_link→{lidar_link, imu_link, base_footprint,
  camera_link→camera_optical_frame}`).

```bash
ros2 launch openamrobot_bringup real_bringup.launch.py
```

### EKF (`config/ekf.yaml`)

Fuses `/odom/unfiltered` (wheel vx, vyaw) + `/imu/data` (**only** `angular_velocity.z`).
The MPU-6500 has no valid orientation quaternion and a tilted accelerometer, so the EKF runs
`two_d_mode: true` with gyro-Z only. `transform_time_offset: 0.2` dates the published TF
forward because the LiDAR scan arrives ~0.14 s ahead of the TF (micro-ROS latency),
preventing "extrapolation into the future" on the SLAM side.

### Measured static TFs (unit-specific)

The LiDAR is mounted **rotated 180°** (`yaw=π`), 0.335 m ahead of the axle, 0.18 m up. The
camera is 0.415 m ahead, 0.12 m up. Re-measure if the sensor mounts change. These describe
**this** robot; on another unit, edit the values in `real_bringup.launch.py`.

> `real_bringup.launch.py` is the **data source only**. For the full stack (data source +
> Nav2), use `bringup.launch.py` above — it adds localization + navigation with the shared
> `openamrobot_nav2/config/nav2_params.yaml` and the right `use_sim_time`.
