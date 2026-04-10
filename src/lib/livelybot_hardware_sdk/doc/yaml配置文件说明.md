# yaml配置文件说明

`rb.Motor`是一个 `vector<motor*>`列表，可以理解为一个装载了机器人上所有的电机接口的一个列表，可以通过这个列表去控制机器人上的电机。但是这个列表的顺序是机器人的配置文件决定的。

* sdk提供了多个机器人配置文件。
* 以12自由度的双足机器人为例，编写配置文件如下：

```
robot:
  SDK_version: 3.0
  robot_name: "Pi Robot"
  arm_dof: 0
  leg_dof: 12
  Serial_Type: "/dev/ttyACM"  # 5361的
  Seial_baudrate: 4000000
  CAN_Type: "CAN-FD BRS" # "CAN-FD"\"CAN-FD BRS"\"CAN 2.0B"
  control_type: 9  # 运行模式
  CANboard_num: 1
  CANboard:
    No_1_CANboard:
      CANport_num: 2
      CANport:
        CANport_1:
          motor_num: 6
          motor:
            motor1:{type: 2, id: 1, name: "L_low_foot", num: 1,
              pos_limit_enable: false, pos_upper: 5, pos_lower: -5,
              tor_limit_enable: false, tor_upper: 5, tor_lower: -3}
            motor2:{type: 2, id: 2, name: "L_high_foot", num: 2,
              pos_limit_enable: false, pos_upper: 5, pos_lower: -5,
              tor_limit_enable: false, tor_upper: 5, tor_lower: -3}
            motor3:{type: 3, id: 3, name: "L_knee_pitch", num: 3,
              pos_limit_enable: false, pos_upper: 5, pos_lower: -5,
              tor_limit_enable: false, tor_upper: 5, tor_lower: -3}
            motor4:{type: 3, id: 4, name: "L_hip_pitch", num: 4,
              pos_limit_enable: false, pos_upper: 5, pos_lower: -5,
              tor_limit_enable: false, tor_upper: 5, tor_lower: -3}
            motor5:{type: 3, id: 5, name: "L_hip_roll", num: 5,
              pos_limit_enable: false, pos_upper: 5, pos_lower: -5,
              tor_limit_enable: false, tor_upper: 5, tor_lower: -3}
            motor6:{type: 3, id: 6, name: "L_hip_yaw", num: 6,
              pos_limit_enable: false, pos_upper: 5, pos_lower: -5,
              tor_limit_enable: false, tor_upper: 5, tor_lower: -3}
        CANport_2:
          motor_num: 6
          motor:
            motor1:{type: 2, id: 1, name: "R_low_foot", num: 1,
              pos_limit_enable: false, pos_upper: 5, pos_lower: -5,
              tor_limit_enable: false, tor_upper: 5, tor_lower: -3}
            motor2:{type: 2, id: 2, name: "R_high_foot", num: 2,
              pos_limit_enable: false, pos_upper: 5, pos_lower: -5,
              tor_limit_enable: false, tor_upper: 5, tor_lower: -3}
            motor3:{type: 3, id: 3, name: "R_knee_pitch", num: 3,
              pos_limit_enable: false, pos_upper: 5, pos_lower: -5,
              tor_limit_enable: false, tor_upper: 5, tor_lower: -3}
            motor4:{type: 3, id: 4, name: "R_hip_pitch", num: 4,
              pos_limit_enable: false, pos_upper: 5, pos_lower: -5,
              tor_limit_enable: false, tor_upper: 5, tor_lower: -3}
            motor5:{type: 3, id: 5, name: "R_hip_roll", num: 5,
              pos_limit_enable: false, pos_upper: 5, pos_lower: -5,
              tor_limit_enable: false, tor_upper: 5, tor_lower: -3}
            motor6:{type: 3, id: 6, name: "R_hip_yaw", num: 6,
              pos_limit_enable: false, pos_upper: 5, pos_lower: -5,
              tor_limit_enable: false, tor_upper: 5, tor_lower: -3}
```

根据以上文件，可以把配置文件分为以下5个部分：

1. 机器人根节点
   * 配置文件以robot:开始;
2. 软硬件参数说明
   * `SDK_version: 3.0` 表示使用的SDK软件版本为3.0;
   * `robot_name: "Pi Robot"` 表示机器人的名字为 `Pi Robot`;
   * `arm_dof:0 `表示机器人的手臂自由度为0，`leg_dof: 12`表示机器人的腿部自由度为12;
   * `Serial_Type: "/dev/ttyACM"` 表示机器人通信板的串口名称为 `/dev/ttyACM*`;
   * `Seial_baudrate:4000000` 表示机器人通信板的串口波特率为4000000;
   * `CAN_TYPE: "CAN-FD"` 表示机器人的CAN总线类型为 `CAN-FD`，可选项有：`"CAN-FD"、"CAN 2.0B"`;
   * `control_type: 9` 表示机器人的控制模式为9，可选项有：`0、1、2、3、4、5、6、9、10（7、8模式已弃用）`，分别对应 `位置模式、速度模式、力矩模式、电压模式、电流模式、位置-速度-最大力矩模式、带Kp-Kd参数的位置-速度-力矩模式、带Kp-Kd参数的位置-速度模式`;
   * `CANboard_num: 1` 表示机器人通信板个数为1；
3. 通信板配置信息
   * `CANboard:` 表示机器人的通信板信息，包含CAN总线数和每条CAN总线上的电机数；
   * `No_1_CANboard:` 表示第一块通信板的节点；
   * `CANport_1:` 表示第一条CAN总线上的电机信息；
   * `motor_num: 6` 表示这条CAN线上的电机个数；
4. 电机配置信息
   * `motor_1:` 表示这条CAN线上的第1个电机信息；
   * `type:1`: 电机类型，可选项有：`1、2、3、4、5、6、7`，分别对应 `5046电机-20减速比、4538电机-20 减速比、5047电机-36 减速比、5047电机-9减速比、4438 电机-32减速比、4438电机，8减速比、7136电机-9减速比`；
   * `id: 1`:电机ID
   * `name: "L_hip_yaw"`:电机名称；
   * `pos_limit_enable: false`:位置保护使能标识，`true`表示使能，`false`表示不使能；
   * `pos_upper: 5`:位置上限，单位是圈数；
   * `pos_lower: -5`:速度下限，单位是圈数；
   * `tor_limit_enable: false`:力矩限制使能标识，`true`表示使能，`false`表示不使能；
   * `tor_upper: 5`:力矩上限，单位是Nm；
   * `tor_lower: -3`:力矩下限，单位是Nm；
