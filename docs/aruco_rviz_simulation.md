# 真实 ArUco 目标 + 仿真自车状态的 RViz 联调

## 数据链路

```text
USB 相机图像
  -> aruco_det
  -> /aruco/pose（camera_optical_frame）
  -> aruco_to_docking_adapter
  -> /qr/tag_measurement（camera_link 轴向）
  -> /qr/tag_yaw_camera（目标前向相对相机航向）
  -> hybrid_target_bridge
  -> /hybrid_target/pose（odom）
  -> /hybrid_target/twist（odom）
  -> odom_docking_controller

demo_sim_node（仅仿真自车运动）
  -> /ego/twist
  -> /ego/yaw
  -> /ego/pose
  -> odom -> ego_base TF

static_transform_publisher
  -> ego_base -> camera_optical_frame

hybrid_target_bridge
  -> /hybrid_target/markers（odom 下 30 Hz 连续目标车体）

odom_docking_controller
  -> /wheel_cmd
  -> demo_sim_node 积分虚拟自车
```

第一条有效二维码用于把目标锁定在 `odom`。之后桥接节点根据虚拟自车轮速和
横摆角产生的运动，持续发布目标在 `odom` 中的位姿，从而形成闭环。
`demo_sim_node` 在本 launch 中只负责虚拟自车，已关闭仿真目标和仿真二维码：

```yaml
sim.publish_qr: false
sim.publish_simulated_target: false
```

二维码安装在目标车后壁中心时：

```text
p_tag_in_target = [-target_length/2, 0]
p_target_center = p_tag_world + R(target_world_yaw) * [target_length/2, 0]
```

启动文件设置 `tag_on_rear_face=true`，桥接节点会按 `0.52 m` 车长
自动得到 `tag_x=-0.26 m`。混合控制器把目标中心距设为：

```text
(target_length + ego_length) / 2 = 0.52 m
```

这对应目标车后壁与自车前壁相贴，而不是两车中心重合。

默认使用二维码平面法向估计目标车辆的真实航向。二维码贴在目标车后壁时，
其平面外法向朝后，因此桥接器把法向反方向定义成目标车前方。`target_world_yaw`
现在只作为没有航向测量时的回退值：

```bash
ros2 launch Docking_pkg docking_aruco_rviz.launch.py \
  video_device:=/dev/video2
```

如果实测目标航向整体反向约 `pi`，增加安装偏角：

```bash
ros2 launch Docking_pkg docking_aruco_rviz.launch.py \
  mounting_yaw_offset:=3.14159265359
```

当二维码中心在 `odom` 中的平移速度超过 `0.06 m/s`，并且连续 5 帧运动
方向一致时，桥接器才使用该运动方向作为目标航向。这里直接使用二维码中心
速度，不使用经过目标航向补偿后的目标中心位置，因此静止二维码的姿态噪声
不会反过来改变目标车朝向。目标停车后保持最后一次可信运动航向。若需要
完全使用二维码法向，可设置：

```bash
prefer_velocity_heading:=false
```

`odom_docking_controller` 的所有位置和航向都在 `odom` 中计算，避免车体系
每周期重新表达目标航向造成跳变。控制流程为：

1. 每个控制周期都从自车当前位置生成一条完整的三次 Bezier 路径；
2. 路径起点的位置和切向为自车当前位姿；
3. 路径终点的位置为目标后方 `0.52 m` 的接触位姿，终点切向直接使用
   二维码测得的目标车辆航向；
4. 控制器在整条路径上跟踪路径切向，因此自车前进过程中航向连续收敛，
   而不是到达目标附近后才旋转；
5. `APPROACH/FINAL_DOCK` 只改变允许速度，不会切换到另一个预对接终点；
6. 只在最后约 `0.15 m` 使用位姿反馈处理数值收尾；
7. `DOCKED`：中心距、横向和航向同时满足收紧后的容差后锁存。

目标运动时不会进入停车回正状态。控制器使用目标速度前馈沿曲线持续跟随，
并在追赶过程中逐步把自车航向收敛到目标轴线。只有已经到达最终位置且目标
静止时，才允许原地完成很小的残余角度修正。

首次对接后若目标再次运动：

- 沿目标轴线的小幅运动直接由 `FINAL_DOCK` 前向跟随；
- 状态显示为 `DOCKED_FOLLOW`，表示已经接触并保持随动；
- 不允许倒车绕到预对接点，避免车尾朝前；
- 横向或距离变化过大时重新进入 `APPROACH`；
- 角速度限制为 `0.60 rad/s`，不会执行整圈旋转。

## 编译

```bash
cd /home/xmz/Dock_ws/aruco_ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select aruco_det
source install/setup.bash

cd /home/xmz/Dock_ws
source /opt/ros/jazzy/setup.bash
source /home/xmz/Dock_ws/aruco_ros2_ws/install/setup.bash
colcon build --symlink-install --packages-select Docking_pkg
source install/setup.bash
```

## 启动

```bash
ros2 launch Docking_pkg docking_aruco_rviz.launch.py \
  video_device:=/dev/video0 \
  primary_id:=0 \
  marker_length:=0.08
```

USB2 相机默认使用该设备验证过的 YUYV 和 `best_effort` 图像 QoS。若需要
减少 USB 带宽，也可以尝试 MJPEG：

```bash
ros2 launch Docking_pkg docking_aruco_rviz.launch.py \
  video_device:=/dev/video2 \
  pixel_format:=mjpeg \
  image_qos_reliability:=best_effort
```

若 MJPEG 运行一段时间后出现 `No new camera images`，应改回
`pixel_format:=yuyv`；部分 USB2 相机驱动会在 MJPEG 取流时停滞。

目标锁定后可让目标沿 odom `+X` 缓慢移动，便于观察完整追赶：

```bash
ros2 launch Docking_pkg docking_aruco_rviz.launch.py \
  video_device:=/dev/video2 \
  live_target_update:=false target_speed:=0.20
```

`target_speed` 只会在 `live_target_update=false` 时生效；默认的
`live_target_update=true` 直接使用真实二维码的连续运动更新目标。

为平滑 5～8 Hz 的 ArUco 检测输出，桥接节点对目标位置和速度做 alpha-beta
滤波，并在两次二维码之间最多外推 `prediction_timeout=0.35 s`。超过该时间
立即停止外推，避免二维码短暂丢失后目标继续“飞走”。

目标锁定后，桥接节点默认继续使用相机中的后续位置变化。这里假设真实相机在
实验过程中位置固定：二维码移动会被解释为目标在 `odom` 中移动，虚拟自车运动
则用于生成对应的相机相对观测。

如果中途移动了相机，或者目标坐标系出现较大变化，需要重新锁定目标：

```bash
ros2 topic pub --once /qr/hybrid/reanchor std_msgs/msg/Bool "{data: true}"
```

然后让二维码重新完整出现在相机中，即可按当前测量重新建立目标 `odom` 位姿。

如果相机已由其他 launch 启动：

```bash
ros2 launch Docking_pkg docking_aruco_rviz.launch.py \
  start_camera:=false
```

相机安装偏移与规划配置保持一致：

```bash
ros2 launch Docking_pkg docking_aruco_rviz.launch.py \
  camera_mount_x:=0.35 camera_mount_y:=0.0 camera_mount_z:=0.20
```

桥接节点使用 launch 参数 `camera_mount_x/y/z` 作为相机安装偏移；目标姿态
发布后由 `odom_docking_controller` 直接使用，不再依赖车体系规划器。

查看 ArUco 检测效果：

```bash
ros2 launch Docking_pkg docking_aruco_rviz.launch.py show_camera_view:=true
```

该参数会另外打开标题为 `ArUco - XYZ in metres` 的 OpenCV 窗口。窗口可能被
RViz 遮挡。默认 RViz 配置还包含 `ArUco Camera` Image 显示，Topic 为
`/aruco_det_image`，也可以直接在 RViz 内查看检测画面。

## 检查顺序

```bash
# 1. 相机图像
ros2 topic hz /usb_cam/image_raw

# 2. 原始 ArUco 位姿，位置在相机光学坐标系下
ros2 topic echo /aruco/pose --once

# 3. 从后壁标记法向估计的目标前向航向
ros2 topic echo /qr/tag_yaw_camera --once

# 4. 桥接后的目标位姿，位于 odom
ros2 topic echo /hybrid_target/pose --once

# 5. 兼容调试用的相机轴向相对位置
ros2 topic echo /qr/tag_position --once

# 6. 仿真自车状态
ros2 topic hz /ego/twist
ros2 topic hz /ego/yaw

# 7. 规划状态
ros2 topic echo /docking/status
```

RViz 的 Fixed Frame 设置成 `odom`。应能看到：

- 绿色车体：仿真自车；
- 红色车体：由 ArUco 位置补偿标签安装偏移后得到的目标；
- 绿色曲线：以目标实时对接位姿为终点的三次 Bezier 路径；
- 黄色球：当前对接点；
- 蓝色预测线：短期目标预测。

混合模式默认关闭 `camera_optical_frame -> aruco_0` TF。原因是该 TF 只在二维码
被检测到的帧发布，会让 RViz 反复出现
`No transform from [aruco_0] to [odom]`。规划不依赖该 TF；目标车体改由桥接
节点以 30 Hz 直接发布到 `/hybrid_target/markers`，固定帧使用 `odom`。

## 常见问题

1. `/aruco/pose` 没有数据：检查 `/dev/video0`、图像话题、相机内参和
   `primary_id`。
2. 有 `/aruco/pose` 但 `/qr/tag_position` 没数据：检查时间戳是否过期，
   以及距离是否在 `[min_range, max_range]` 内。
3. 红色目标位置偏差一个车身：确认
   `position_processor.tag_on_rear_face=true`，并检查实际车长是否为
   `vehicle.target_length=0.52`、`vehicle.ego_length=0.52`。
4. 目标左右反向或上下异常：确认适配器使用
   `axis_mode=optical_to_camera_link`。
5. 轨迹乱跳：先确认 `/usb_cam/image_raw` 和 `/qr/tag_position` 的时间戳连续，
   再检查相机内参和 ArUco 黑色方块尺寸 `0.08 m`。
6. TF 中不要以 `aruco_0` 作为显示目标帧。混合 launch 默认设置
   `publish_aruco_tf=false`，因此不会持续提供该间歇帧；目标车体显示应使用
   `odom` 下的 `/hybrid_target/markers`。

## 当前约束

- ArUco 必须至少提供一帧包含目标 ID 的清晰图像；
- 目标车辆当前仍要求在自车前方做直线运动；
- 自车轮速和横摆角由仿真节点生成，不是真实底盘测量；
- 相机内参文件 `config/usb_camera.yaml` 标记为未重新标定，实机使用前应完成标定；
- 二维码安装偏移、相机安装偏移和对接中心距必须按实车测量。
