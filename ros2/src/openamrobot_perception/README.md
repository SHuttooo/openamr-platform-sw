# openamrobot_perception

Perception modules for OpenAMRobot. Currently provides the **real-robot LiDAR body
filter**; object detection, point-cloud, and fiducial perception may be added here later.

## `scan_body_filter` node

Removes the robot's own chassis reflections from the LiDAR so they are not treated as
obstacles, then republishes `/scan` as `/scan_filtered`.

This is the **real-robot** filter. It differs from the **simulation** filter
(`openamrobot_nav2/config/scan_body_filter.yaml`, a `laser_filters` angular chain) in two
ways that matter on hardware: it masks **by distance** (keeps real walls behind thin side
posts) and it publishes **RELIABLE** so the Nav2 costmaps receive the scan. The two are a
**sim / real profile pair**, not duplicates.

```bash
ros2 launch openamrobot_perception scan_body_filter.launch.py
# or, with a different calibration:
ros2 launch openamrobot_perception scan_body_filter.launch.py params_file:=/path/to.yaml
```

### Parameters

Defaults are calibrated for **this unit's** LiDAR mount (RPLIDAR A1 mounted rotated
180 deg: 0 deg = robot rear, +/-180 deg = front, -90 = left, +90 = right). Re-measure
(watch `/scan` in RViz) if the mount, chassis, or URDF changes. Values live in
[`config/scan_body_filter_real.yaml`](config/scan_body_filter_real.yaml).

| Parameter | Type | Default | Unit | Meaning / impact |
|---|---|---|---|---|
| `scan_in` | string | `/scan` | topic | input LaserScan |
| `scan_out` | string | `/scan_filtered` | topic | filtered output |
| `reliable_qos` | bool | `true` | — | publish RELIABLE; **false leaves Nav2 costmaps empty** |
| `close_max` | double | `0.40` | m | in "close" sectors, only returns nearer than this are removed |
| `full_mask_sectors_deg` | double[] | `[-45, 49]` | deg | sectors removed at ALL distances (flat `lo,hi,…` pairs) |
| `close_mask_sectors_deg` | double[] | `[-96,-73, 73,96]` | deg | sectors where only `< close_max` returns are removed |

**Failure modes:** angles too wide -> real walls near the body get blanked (robot blind to
close obstacles); `reliable_qos: false` -> costmaps stay empty; wrong frame convention (mount
not rotated 180 deg) -> masks the wrong side.

### Topics / TF

- Subscribes: `scan_in` (`sensor_msgs/LaserScan`, SensorData QoS).
- Publishes: `scan_out` (`sensor_msgs/LaserScan`, RELIABLE by default).
- No TF is published; the output keeps the input `header.frame_id` (the LiDAR frame).
