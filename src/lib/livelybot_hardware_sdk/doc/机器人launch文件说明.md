# 机器launch文件说明

launch文件作用是在一个脚本文件内启动若干个ros节点，定义ros参数。

1. 控制机器人上的电机，主要使用roslaunch脚本。sdk的roslaunch脚本主要放在livelybot_robot/src/livelybot_bringup/launch路径下，以motor_move_zero.launch脚本为例，编写launch文件如下：

```
<launch>
  <include file='$(find livelybot_description)/launch/livelybot_description_robot.launch' />
  <node pkg="livelybot_bringup" name="motor_move_zero" type="motor_move_zero" output="screen" />
</launch> 
```

2. 其中，第二行表示引用了livelybot_description包下launch目录下的livelybot_description_robot.launch脚本,而livelybot_description_robot.launch脚本内容如下:
    ```
    <launch>
        <arg name = "dof_type" default = "12"/>
        <arg name = "mcu_type" default = "STM32H730"/>
        <arg name = "model_type" default = "test"/>
        <arg name = "design" default = "Orin"/> 
        <rosparam file="$(find livelybot_description)/robot_param/$(arg dof_type)dof_$(arg mcu_type)_model_$(arg model_type)_$(arg design)_params.yaml" command="load" />
    </launch>
    <!-- 注释
    pkg： 节点所在的功能包名称
    type: 节点的可执行文件名称
    name: 节点运行时的名称
    output  ="log | screen" (可选)，日志发送目标，可以设置为 log 日志文件，或 screen 屏幕,默认是 log
    respawn ="true | false" (可选)，如果节点退出，是否自动重启
    required="true | false" (可选)，该节点是否必须，如果为 true,那么如果该节点退出，将杀死整个 roslaunch
    ns="xxx" (可选)，在指定命名空间 xxx 中启动节点
    machine="机器名"，在指定机器上启动节点
    args="xxx xxx xxx" (可选)，将参数传递给节点
    -->
    ```
    其核心内容是：
    ```
    <rosparam file="$(find livelybot_description)/robot_param/$(arg dof_type)dof_$(arg mcu_type)_model_$(arg model_type)_$(arg design)_params.yaml" command="load" />
    ```
    这一行内容表示把提供的机器人配置文件内的参数导入到ros的工作空间内，那么在代码里的robot类初始化时，便可以读取到配置文件里的参数，那么就可以使用一套代码，适应不同的机器人电机的连接方式以及电机数量。
3. 第三行表示在livelybot_bringup包下启动motor_move_zero节点，并输出到屏幕。功能上让电机运动到每个关节零位。
  ```
  <node pkg="livelybot_bringup" name="motor_move_zero" type="motor_move_zero" output="screen" />
  ```