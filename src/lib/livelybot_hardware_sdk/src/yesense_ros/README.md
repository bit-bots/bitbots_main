## Usage

### Build

Build with the rest of the workspace using colcon or pixi:

```bash
pixi run build
```

### Run

To change the serial port or baud rate, edit the parameter file at
`src/yesense/config/yesense_params.yml` — no recompilation needed.

```bash
ros2 launch yesense_imu yesense.launch.xml
```

To inspect raw sensor output:

```bash
ros2 topic echo /imu/original_data
```

## Supported ROS 2 Topics

### Standard topics

| Topic | Description |
| :---- | :---- |
| `/imu/data` | IMU acceleration data |
| `/imu/marker` | IMU attitude, position, and shape data (for RViz visualization) |
| `/imu/paths` | Sensor motion path (no data yet) |
| `/pose` | Sensor pose data |

### Yesense extension topics

| Topic | Description | Notes |
| :---- | :---- | :---- |
| `/yesense/command_resp` | Yesense command response | Subscribe to this topic to receive the return value when publishing a command via `ros2 topic pub` |
| `/yesense/sensor_data` | Sensor data | Includes: temperature, accel, gyro, euler angles, quaternion, position, timestamp |
| `/yesense/gnss_data` | Raw GNSS data | |
| `/yesense/gps_data` | Raw NMEA0183 GPS sentences | |
| `/yesense/imu_status` | Sensor status (fusion state, GNSS fix state) | |
| `/yesense/all_data` | All data (union of all `/yesense/*` topics) | |
| `/imu/original_data` | Alias for `/yesense/all_data` (backwards compatibility) | |

### Yesense configuration topics

These topics are used to configure sensor functions and read back their status.

Monitor responses by running:

```bash
ros2 topic echo /yesense/command_resp
```

Example response:

```
---
id: "yesense/gyro_bias_estimate"
cmd: "onn"
success: False
msg: "Invalid command: 'onn'"
data: []
```

Response fields:

| Field | Type | Description |
| :---- | :---- | :---- |
| `id` | string | Topic name |
| `cmd` | string | Command name |
| `success` | bool | Whether the command succeeded |
| `msg` | string | Status message; contains error description on failure |
| `data` | uint8[] | Raw bytes returned by the command |

**1. Gyro bias auto-estimation**

Topic: `/yesense/gyro_bias_estimate`

- Enable bias estimation:
  ```bash
  ros2 topic pub /yesense/gyro_bias_estimate std_msgs/msg/String "{data: enable}"
  ```

  Example response:
  ```
  ---
  id: "yesense/gyro_bias_estimate"
  cmd: "enable"
  success: True
  msg: "ok"
  data: [0]
  ```

- Disable bias estimation:
  ```bash
  ros2 topic pub /yesense/gyro_bias_estimate std_msgs/msg/String "{data: disable}"
  ```

- Query status:
  ```bash
  ros2 topic pub /yesense/gyro_bias_estimate std_msgs/msg/String "{data: query}"
  ```

  Example response:
  ```
  ---
  id: "yesense/gyro_bias_estimate"
  cmd: "query"
  success: True
  msg: "ok"
  data: [81, 2]
  ```
