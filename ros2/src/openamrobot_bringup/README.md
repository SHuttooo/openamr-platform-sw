# openamrobot_bringup

Top-level launch composition for OpenAMRobot. Wires subsystems together; contains no
low-level nodes, models, or Nav2 parameters of its own (those live in their packages).

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

> Pair this with `openamrobot_nav2`'s **real** profile (`nav2_params_real.yaml` +
> `real_bringup_launch.py`, `use_sim_time:=false`).
