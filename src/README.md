## 目录：

1. Moveit fake controller运行
2. 导航+Moveit+Gazebo+仿真环境运行
3. Moveit独立控制真实机械臂
4. Movet联合控制真实机械臂+底盘规划
5. Moveit独立控制真实onRobot夹爪
6. Moveit联合控制真实机械臂+onRobot夹爪
7. Moveit联合控制真实机械臂+onRobot夹爪+底盘
8. Gazebo仿真环境demo的运行
9. 真实机器人demo的运行
10. 参数的说明 
11. 问题说明

## fake controller运行
```shell
roslaunch kmriiwa_moveit move_group.launch fake_execution:=true load_realsense_topic:=false merge_real_RG2FT_kmriiwa:=false load_simulation:=false load_real_RG2FT:=false load_real_kmriiwa:=false load_realsense:=false load_real_move_base:=false
```

- 在rviz的Motion Planning中选择kmriiwa_manipulator的Planning Group即可进行机械臂的fake规划，选择gripper的Planning Group即可进行onRobot夹爪的fake规划，后面的运行同理。
##   导航+Moveit+Gazebo仿真环境运行
```shell
roslaunch kmriiwa_gazebo gazebo_pick_place_demo.launch  load_realsense:=false 
#启动gazebo
roslaunch kmriiwa_bringup planning_stack_bringup.launch load_realsense_topic:=false merge_real_RG2FT_kmriiwa:=false load_simulation:=true load_real_RG2FT:=false load_real_kmriiwa:=false load_realsense:=false load_move_group_rviz:=false load_real_move_base:=false
#启动move_group/导航/rviz

#如果不需要启动导航，只启动move_group/rviz，则:
roslaunch kmriiwa_moveit move_group.launch load_realsense_topic:=false merge_real_RG2FT_kmriiwa:=false load_simulation:=true load_real_RG2FT:=false load_real_kmriiwa:=false load_realsense:=false load_real_move_base:=false load_move_group_rviz:=true
```

- 选中打开rviz上方Panels中的Tools，点击2D Nav Goal可进行导航规划，点击2D Pose Estimate可进行机器人在map地图中的定位。
## Moveit独立控制真实机械臂
```shell
roslaunch kmriiwa_moveit move_group.launch load_realsense_topic:=false merge_real_RG2FT_kmriiwa:=false load_simulation:=false load_real_RG2FT:=false load_real_kmriiwa:=true load_realsense:=true load_real_move_base:=false
```
## Moveit联合控制真实机械臂+底盘
```shell
roslaunch kmriiwa_bringup planning_stack_bringup.launch load_realsense_topic:=false merge_real_RG2FT_kmriiwa:=false load_simulation:=false load_real_RG2FT:=false load_real_kmriiwa:=true load_realsense:=true load_real_move_base:=true

```
## Moveit独立控制真实onRobot夹爪
```shell
roslaunch kmriiwa_moveit move_group.launch load_realsense_topic:=false merge_real_RG2FT_kmriiwa:=false load_simulation:=false load_real_RG2FT:=true load_real_kmriiwa:=false load_realsense:=true load_real_move_base:=false

```
## Moveit联合控制真实机械臂+onRobot夹爪
```shell
roslaunch kmriiwa_moveit move_group.launch load_realsense_topic:=false merge_real_RG2FT_kmriiwa:=true load_simulation:=false load_real_RG2FT:=false load_real_kmriiwa:=false load_realsense:=true load_real_move_base:=false
```
## Moveit联合控制真实机械臂+onRobot夹爪+底盘
```shell
roslaunch kmriiwa_bringup planning_stack_bringup.launch load_realsense_topic:=false merge_real_RG2FT_kmriiwa:=true load_simulation:=false load_real_RG2FT:=false load_real_kmriiwa:=false load_realsense:=true load_real_move_base:=true
```
## 仿真环境demo运行
```shell
roslaunch kmriiwa_gazebo gazebo_pick_place_demo.launch  load_realsense:=false
#启动gazebo
roslaunch kmriiwa_bringup planning_stack_bringup.launch load_realsense_topic:=false merge_real_RG2FT_kmriiwa:=false load_simulation:=true load_real_RG2FT:=false load_real_kmriiwa:=false load_realsense:=false load_move_group_rviz:=false load_real_move_base:=false
#启动move_group/导航/rviz
roslaunch kmriiwa_simulation_grasp find_object.launch load_realsense:=false load_real_world:=false
#目标检测，获取目标的位姿
rosrun deep_grasp_task tf_transform_cylinder.py
#发布目标位姿tf
roslaunch kmriiwa_simulation_grasp demo.launch  load_realsense:=false load_real_world:=false
#启动demo运行程序
```
## 真实机器人demo运行
```shell
roslaunch kmriiwa_bringup planning_stack_bringup.launch load_realsense_topic:=true merge_real_RG2FT_kmriiwa:=true load_simulation:=false load_real_RG2FT:=false load_real_kmriiwa:=false load_realsense:=true load_move_group_rviz:=false load_real_move_base:=true
#启动move_group/导航/rviz
roslaunch kmriiwa_simulation_grasp find_object.launch load_realsense:=true load_real_world:=true
#目标检测，获取目标的位姿
rosrun deep_grasp_task tf_transform_cylinder.py
#发布目标位姿tf
roslaunch kmriiwa_simulation_grasp demo.launch  load_realsense:=false load_real_world:=true
#启动demo运行程序
```
## 参数的说明

1. 如果想改变导航的速度，需访问：
```shell
kmriiwa_ros_stack/kmriiwa_navigation/config/local_planner.yaml
```

2. 要想改变局部和全局地图障碍物的半径，需访问：
```shell
src/kmriiwa_ros_stack/kmriiwa_navigation/config/local_costmap.yaml
src/kmriiwa_ros_stack/kmriiwa_navigation/config/global_costmap_static_map.yaml
```

3. 若相机在机器人上的位置发生变化，下面的文件用于改变运行仿真以及真实机器人demo时，相机坐标相对于base坐标的变换
```shell
src/deep_grasp_demo/deep_grasp_task/config/calib/camera_real.yaml
src/deep_grasp_demo/deep_grasp_task/config/calib/camera_sim.yaml
```

4. 下面的文件用于调整运行仿真以及真实机器人demo时的参数
```shell
src/deep_grasp_demo/deep_grasp_task/config/kmriiwa_object_real.yaml
src/deep_grasp_demo/deep_grasp_task/config/kmriiwa_object_sim.yaml
```
其中，主要调参的参数为：
```shell
1.Scene fames：用于设定参考坐标系
2.move_base_target：用于设定导航相对于参考坐标系的位置
3.Spawn_cylinder和Spawn_table为true，需要加载这两个物体，调整其形状与位姿
4.demo运行时，文件cylinder的位姿设定无作用，其位姿在程序中由目标检测结果订阅而来
5.grasp_frame_transform：决定了机械臂抓取到目标前的最后的位姿，即hand_frame相对于cylinder_frame的位姿
6.cylinder_place_pose：决定了放置cylinder时相对于参考坐标系的位置
7.approach_object_min/max_dist：指第5点的位姿不是直接就到达的，而是先到达一个与第5点位姿相同的姿态，然后以该参数的距离移动到第5点的位姿
8. lift_object_min/max_dist：决定了拿起物体时的高度距离
```
## 问题说明

1. 仿真时，realsense和kinect相机都可使用，但realsense的点云由深度图转换裁剪而来，占用资源比kinect大，比较卡顿，且仿真的realsense相机后来参考了真实相机的姿态进行连接，其位姿更改后，不能直接适用于仿真，若要使用并运行demo，需调整相机的位姿以及相关demo参数，建议使用kinect，即load_realsense:=false。
2. 真实机器人用的realsense相机，但在demo.launch中load_realsense:=false，是因为load_realsense的作用在此是为了区分话题名称，load_realsense:=true会加载realsense仿真时的点云话题，false会加载kinect仿真时的点云话题，而真实的realsense的话题名称设置的与仿真时kinect一致，所以直接加载仿真kinect话题。
