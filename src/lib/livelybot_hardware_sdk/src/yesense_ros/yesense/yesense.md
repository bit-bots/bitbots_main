
## 1. Usage

```bash
ros2 launch yesense_imu yesense.launch.xml
```

Example launch file:

```xml
<launch>
  <node pkg="yesense_imu" exec="yesense_imu_node" name="yesense_node" output="screen">
    <param name="yesense_port"     value="/dev/ttyUSB0"/>
    <param name="yesense_baudrate" value="460800"/>
  </node>
</launch>
```

`yesense_port` is the serial device path; `yesense_baudrate` is the baud rate.



## 2. Parameter command reference

### 2.1 Queries

#### 2.1.1 Product information (0x00)

1. Query software version

```bash
ros2 topic pub /production_query std_msgs/msg/Int8 "{data: 1}"
```

2. Query product model

```bash
ros2 topic pub /production_query std_msgs/msg/Int8 "{data: 2}"
```

#### 2.1.2 Query baud rate (0x02)

```bash
ros2 topic pub /baudrate_query std_msgs/msg/Empty "{}"
```

#### 2.1.3 Query output frequency (0x03)

```bash
ros2 topic pub /freequency_query std_msgs/msg/Empty "{}"
```

#### 2.1.4 Query output content (0x04)

```bash
ros2 topic pub /output_content_query std_msgs/msg/Empty "{}"
```

#### 2.1.5 Query standard parameters (0x05)

1. Query gyro user bias

   ```bash
   ros2 topic pub /standard_param_query std_msgs/msg/UInt8 "{data: 1}"
   ```

2. Read static threshold

   ```bash
   ros2 topic pub /standard_param_query std_msgs/msg/UInt8 "{data: 2}"
   ```

#### 2.1.6 Query algorithm mode (0x4D)

1. Algorithm mode

```bash
ros2 topic pub /mode_query std_msgs/msg/UInt8 "{data: 1}"
```

2. Dynamic mode query

```bash
ros2 topic pub /mode_query std_msgs/msg/UInt8 "{data: 2}"
```

3. SYNC OUT query

```bash
ros2 topic pub /mode_query std_msgs/msg/UInt8 "{data: 3}"
```



### 2.2 Volatile configuration (lost on power-off)

If the MSB of `data` is 1, the setting is volatile (RAM only). If the MSB is 0, the setting is saved to flash.

#### 2.2.1 Reset all parameters (0x01)

Deprecated.

#### 2.2.3 Set baud rate (0x02)

Low 4 bits select the baud rate value.

| Value | Baud rate |
| :---- | :-------- |
| 0x81  | 9600      |
| 0x82  | 38400     |
| 0x83  | 115200    |
| 0x84  | 460800    |
| 0x85  | 921600    |
| 0x86  | 19200     |
| 0x87  | 57600     |
| 0x88  | 76800     |
| 0x89  | 230400    |

```bash
ros2 topic pub /baudrate_setting std_msgs/msg/UInt8 "{data: 0x84}"
```

#### 2.2.3 Set output frequency (0x03)

| Value | Frequency |
| :---- | :-------- |
| 0x81  | 1 Hz      |
| 0x82  | 2 Hz      |
| 0x83  | 5 Hz      |
| 0x84  | 10 Hz     |
| 0x85  | 20 Hz     |
| 0x86  | 25 Hz     |
| 0x87  | 50 Hz     |
| 0x88  | 100 Hz    |
| 0x89  | 200 Hz    |

```bash
ros2 topic pub /freequency_setting std_msgs/msg/UInt8 "{data: 0x88}"
```

#### 2.2.4 Set output content (0x04)

1. Output nothing

   ```bash
   ros2 topic pub /output_content_setting std_msgs/msg/UInt8 "{data: 0x80}"
   ```

2. Accel, gyro, mag, euler, quaternion

   ```bash
   ros2 topic pub /output_content_setting std_msgs/msg/UInt8 "{data: 0x81}"
   ```

3. Position, velocity, UTC, accel, gyro, mag, euler, quaternion

   ```bash
   ros2 topic pub /output_content_setting std_msgs/msg/UInt8 "{data: 0x82}"
   ```

#### 2.2.5 Standard parameter settings (0x05)

1. Reset gyro user bias

   ```bash
   ros2 topic pub /standard_param_setting std_msgs/msg/UInt8 "{data: 0x80}"
   ```

#### 2.2.6 Set algorithm mode (0x4D)

| Value | Mode              |
| :---- | :---------------- |
| 0x81  | AHRS              |
| 0x82  | VRU               |
| 0x83  | IMU               |
| 0x84  | GenerPos          |
| 0x85  | AutoMotive        |
| 0x86  | Data ready        |
| 0x87  | PPS               |
| 0x88  | General mode      |
| 0x89  | Quadruped robot   |
| 0x8A  | Embedded bias cal |

```bash
ros2 topic pub /mode_setting std_msgs/msg/UInt8 "{data: 0x81}"
```



### 2.3 Persistent configuration (saved to flash)

Same commands as section 2.2 but with MSB = 0.

#### 2.3.1 Reset all parameters (0x01)

Deprecated.

#### 2.3.2 Set baud rate (0x02)

| Value | Baud rate |
| :---- | :-------- |
| 0x01  | 9600      |
| 0x02  | 38400     |
| 0x03  | 115200    |
| 0x04  | 460800    |
| 0x05  | 921600    |
| 0x06  | 19200     |
| 0x07  | 57600     |
| 0x08  | 76800     |
| 0x09  | 230400    |

#### 2.3.3 Set output frequency (0x03)

| Value | Frequency |
| :---- | :-------- |
| 0x01  | 1 Hz      |
| 0x02  | 2 Hz      |
| 0x03  | 5 Hz      |
| 0x04  | 10 Hz     |
| 0x05  | 20 Hz     |
| 0x06  | 25 Hz     |
| 0x07  | 50 Hz     |
| 0x08  | 100 Hz    |
| 0x09  | 200 Hz    |

#### 2.3.4 Set output content (0x04)

1. Output nothing

   ```bash
   ros2 topic pub /output_content_setting std_msgs/msg/UInt8 "{data: 0x00}"
   ```

2. Accel, gyro, mag, euler, quaternion

   ```bash
   ros2 topic pub /output_content_setting std_msgs/msg/UInt8 "{data: 0x01}"
   ```

3. Position, velocity, UTC, accel, gyro, mag, euler, quaternion

   ```bash
   ros2 topic pub /output_content_setting std_msgs/msg/UInt8 "{data: 0x02}"
   ```

#### 2.3.5 Standard parameter settings (0x05)

1. Reset attitude angle to zero

   ```bash
   ros2 topic pub /standard_param_setting std_msgs/msg/UInt8 "{data: 0x01}"
   ```

2. Reset heading to zero

   ```bash
   ros2 topic pub /standard_param_setting std_msgs/msg/UInt8 "{data: 0x02}"
   ```

3. Reset gyro user bias

   ```bash
   ros2 topic pub /standard_param_setting std_msgs/msg/UInt8 "{data: 0x03}"
   ```

#### 2.3.6 Set algorithm mode (0x4D)

| Value | Mode              |
| :---- | :---------------- |
| 0x01  | AHRS              |
| 0x02  | VRU               |
| 0x03  | IMU               |
| 0x04  | GenerPos          |
| 0x05  | AutoMotive        |
| 0x06  | Data ready        |
| 0x07  | PPS               |
| 0x08  | General mode      |
| 0x09  | Quadruped robot   |
| 0x0A  | Embedded bias cal |
