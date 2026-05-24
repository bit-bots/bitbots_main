# Biped Robot Control Interface Package

---

## I. Introduction

This is a software interface package for biped robot control and communication. Through this package, you can easily communicate with the underlying hardware of a biped robot to retrieve status information and control motor actions.

---

## II. Installing Dependencies

1. **Install ROS** — using fishros one-click installer (desktop version recommended):
   ```bash
   wget http://fishros.com/install -O fishros && . fishros
   ```

2. **Install serial communication packages:**
   ```bash
   sudo apt-get install libserialport0 libserialport-dev
   ```

3. **Install Python dependencies:**
   ```bash
   python3 -m pip install empy
   ```

---

## III. Building

1. Clone the repository:
   ```bash
   git clone https://github.com/HighTorque-Robotics/livelybot_robot.git
   ```

2. Build:
   ```bash
   cd livelybot_robot
   catkin_make
   ```

---

## IV. Testing

1. **IMU Device Test**
   - Grant execute permission: `chmod +x test_yesense_imu.sh`
   - Run the test script: `./test_yesense_imu.sh`
   - Observe the IMU orientation in the launched RVIZ interface.

2. **Motor Control Test 1**
   - Prepare a main control board and motors.
   - Based on the config file `lively_description/robot_param/1dof_STM32H730_model_test_Orin_params.yaml`, connect 1 motor on `can0` with motor ID `1`.
   - In `livelybot_description.launch`, set `dof_type` to `1`.
   - Grant permissions: `chmod -R 777 test_motor_run.sh`
   - Run the motor test: `./test_motor_run.sh`

3. **Motor Control Test 2**
   - Prepare a main control board and motors.
   - Based on `6dof_STM32H730_model_test_Orin_params.yaml`, connect 3 motors on `can0` (IDs: `1, 2, 3`) and 3 motors on `can1` (IDs: `1, 2, 3`).
   - Set `dof_type` to `6` in `livelybot_description.launch`.
   - Run: `./test_motor_run.sh`

4. **Motor Feedback Test**
   - Same hardware setup as Test 2.
   - Set `dof_type` to `6`.
   - Run: `./test_motor_feedback.sh`

5. **Back Panel Display Test**
   - Connect the back panel display hardware, then run:
     ```bash
     rosrun livelybot_oled livelybot_oled_hd_test
     ```
   - All display screens should show normal content.

---

## V. Reading IMU Data

1. In the `livelybot_robot` directory, run: `source devel/setup.bash`
2. Check the device name: `ls /dev` — typically `ttyACM*` or `ttyUSB*`
3. Grant device permissions: `sudo chmod -R 777 /dev/ttyACM*`
4. Launch the IMU node: `roslaunch yesense_imu run_without_rviz.launch`
5. List published topics: `rostopic list`
6. View IMU orientation data: `rostopic echo /yesense/sensor_data`
7. For a graphical interface: `roslaunch yesense_imu yesense_rviz.launch`

---

## VI. Motor Control <span id="motor_control"></span>

1. Connect the main control board and check its device name: `ls /dev`
2. Grant permissions: `sudo chmod -R 777 /dev/ttyACM*`
3. Write a config file — refer to `lively_description/robot_param/6dof_STM32H730_model_test_Orin_params.yaml` and the [YAML config file guide](./doc/yaml配置文件说明.md)
4. Create a robot object: `livelybot_serial::robot rb`
5. The member variable `rb.Motors` is a container holding all motor objects described in the config file. Motors are ordered as: `canport0` motors 1, 2, 3, …; then `canport1` motors 1, 2, 3, …; then `canport3`, `canport4`, etc.
6. Get a motor object:
   ```cpp
   auto it = rb.Motors.begin();
   motor *m = &(*it + N);
   ```
   Where `N` is the zero-based index of the motor in the container.

7. **Send control commands** (stores command in buffer):
   ```cpp
   m->fresh_cmd_int16(pos, vel, tor, kp, ki, kd, acc, voltage, current); // Debug/general mode (set working mode in config; only relevant params apply)
   m->position(pos);                    // Position mode
   m->velocity(vel);                    // Velocity mode
   m->torque(tor);                      // Torque mode
   m->voltage(volt);                    // Voltage mode
   m->pos_vel_MAXtqe(pos, vel, tor_upper);         // Position+velocity with torque limit
   m->pos_vel_tqe_kp_kd(pos, vel, tor, kp, kd);   // PD position+velocity + feedforward torque
   m->pos_vel_kp_kd(pos, vel, kp, kd);             // PD position+velocity mode
   ```

8. **Transmit commands** to the motor controller board: `rb.motor_send_2()`

---

## VII. Reading Motor State

1. Follow steps 1–5 from the Motor Control section above.
2. Get a motor object:
   ```cpp
   auto it = rb.Motors.begin();
   motor *m = &(*it + N);
   ```
3. Read motor state: `m->get_current_motor_state()` — returns a `motor_back_t *` with the following fields:
   ```cpp
   typedef struct motor_back_struct {
       uint8_t ID;     // Motor ID
       float position; // Position
       float velocity; // Velocity
       float torque;   // Torque
   } motor_back_t;
   ```

---

## VIII. Usage Examples <span id="motor_use_sample"></span>

- Motor control example: `./livelybot_serial/test/test_motor_run.cpp`
- Motor feedback example: `./livelybot_serial/test/test_motor_feedback.cpp`

---

## IX. Motor Protection

1. **Position Protection**
   Add `pos_upper`, `pos_lower`, and `pos_limit_enable` under the `motor*` tag in the config file:
   ```yaml
   motor1:
       type: 1
       id: 1
       name: "L_low_foot"
       num: 1
       pos_limit_enable: true
       pos_upper: 10
       pos_lower: -10
   ```
   When `pos_limit_enable` is `true`, position is clamped between `pos_lower` and `pos_upper`.

2. **Torque Protection**
   Add `tor_upper`, `tor_lower`, and `tor_limit_enable`:
   ```yaml
   motor1:
       type: 1
       id: 1
       name: "L_low_foot"
       num: 1
       tor_limit_enable: true
       tor_upper: 5
       tor_lower: -3
   ```
   When `tor_limit_enable` is `true`, torque is clamped between `tor_lower` and `tor_upper`.

---

## X. Controlling the Robot

1. Write or use a YAML config file — refer to `6dof_STM32H730_model_test_Orin_params.yaml` and the [YAML config guide](./doc/yaml配置文件说明.md).
2. Write a ROS node with a `robot` class instance for motor control and state reading — see [Motor Control](#motor_control) or [Usage Examples](#motor_use_sample).
3. Write a launch file that loads the YAML config and starts the node — see the [Robot Launch File Guide](./doc/机器人launch文件说明.md).
4. Grant serial port permissions:
   ```bash
   sudo chmod -R 777 /dev/ttyACM*
   ```
5. Launch the launch file.

---

## XI. Software Power On/Off

In addition to the hardware switch, the power board can be powered on/off via CAN commands. In this SDK, this feature is wrapped as a ROS topic. Send power control commands via the `/power_switch_control` topic (note: devices powered by the power board can only send a **shutdown** command).

Example:
```cpp
#include <livelybot_power/Power_switch.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "power_switch_control");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<livelybot_power::Power_switch>("/power_switch_control", 10);
    livelybot_power::Power_switch msg;
    while (ros::ok())
    {
        msg.control_switch = 0; // Main controller power: 0 = off, 1 = on
        msg.power_switch = 0;   // Motor power: 0 = off, 1 = on
        pub.publish(msg);
        ros::Duration(1).sleep();
        ros::spinOnce();
    }
}
```

In `livelybot_power::Power_switch`: `control_switch = 0` cuts main controller power, `1` connects it; `power_switch = 0` cuts motor power, `1` connects it.
