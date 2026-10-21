# Docking_pkg

“定位 XY → 目标状态估计 → 目标运动预测 → 动态参考线 → Frenet局部规划 → 速度规划 → 对接状态机 → 控制执行”的滚动轨迹规划闭环。

当前版本不考虑静态/动态障碍物，只使用目标车辆的实时 XY 坐标，因此目标状态估计采用常速度卡尔曼滤波，从位置序列在线估计目标速度、航向和横摆角速度。

## 模块

| 文件 | 职责 |
| --- | --- |
| `target_state_estimator` | 4 状态常速度 Kalman，估计目标 `x, y, vx, vy`，再由速度估计航向/横摆角速度 |
| `target_predictor` | CTRV（Constant Turn Rate and Velocity）预测，`omega` 真正参与目标未来位置与朝向积分 |
| `reference_line_generator` | 动态对接时间 `td`：从预测轨迹中搜索自车在自身速度/加速度/减速度约束下可到达的最早对接位姿 |
| `local_planner` | 支持 Cartesian 和 Frenet 局部规划；Frenet 偏移曲线使用 `dl/ds`，起点横向导数来自自车与参考线的实际航向差，并计算真实偏移曲率 |
| `speed_planner` | 沿路径生成速度剖面：最大加速度、最大减速度、终点不越界、加速度输出，并支持目标到达时间 `td` 约束 |
| `docking_state_machine` | 根据相对纵向距离和目标速度计算追赶速度：目标在前方时施加追赶余量，接近后收敛到目标速度，完成末端对接 |
| `differential_controller` | 纯追踪/PI 式前轮转弯曲率到差速轮速，输出左/右轮角速度 |
| `docking_planner_node` | ROS 2 节点，订阅 odom 下的自车/目标位姿，滚动重规划、发布路径/轨迹/状态/轮速 |
| `qr_target_adapter_node` | 二维码适配节点：把 `camera -> tag` 相对位姿通过 TF 转换到 `odom -> target_base`，再发布 `/target/pose` |
| `demo_sim_node` | 完整仿真节点：模拟目标运动、自车差速运动、带噪声的“定位 XY”输出，发布车体/锥套可视化 Marker 与 TF |

## 几何参数

- 平台长度：0.52 m
- 平台宽度：0.45 m
- 左右轮距：0.60 m
- 轮胎半径：10 inch = 0.254 m
- 圆锥对接母线：0.08 m
- 对接深度：0.05 m
- 底盘高度/车身高：0.20 m（控制暂不使用）

由上述参数得到的理想中心距：

`target_length/2 + ego_length/2 m`

动态对接时间 `td` 的选取方式：在目标 CTRV 预测轨迹上逐点计算目标对接位姿，并用 `minimumTravelTime(effective_distance, v_ego, v_target, a_max, d_max, v_max)` 判断自车能否在该时刻之前到达。`effective_distance` 在直线距离基础上加入少量转角惩罚。选择第一个满足约束的预测点作为本次滚动规划的对接目标，`SpeedPlanner` 再按 `td` 生成满足目标到达时间约束的速度剖面。

## 话题

输入：

- `/qr/tag_pose`：`geometry_msgs/msg/PoseStamped`，二维码标签相对相机的位姿，`header.frame_id` 应为相机坐标系
- `/ego/pose`：`geometry_msgs/msg/PoseStamped`，自车在 `odom` 下的位姿
- `/ego/twist`：`geometry_msgs/msg/TwistStamped`，自车在 `odom` 下的速度

二维码适配节点输出：

- `/target/pose`：目标车底盘在 `odom` 下的位姿
- `/target/twist`：可选；默认关闭，因为二维码只给相对位姿时，直接差分速度噪声较大

如果定位模块直接提供速度，规划节点会优先使用 twist；没有 twist 时才用位姿差分作为降级方案。仿真节点 `demo_sim_node` 会同时发布坐标和速度。

在没有真实定位模块时，`demo_sim_node` 会生成模拟真值位姿，并在其 XY 上加入高斯噪声后发布到上述两个话题，用来模拟“定位模块只给坐标”的输入条件。目标位姿中的朝向会被置 0，因为规划器原本就假设不知道目标朝向，只由 Kalman 从 XY 序列中估计。

输出：

- `/planning/path`：`nav_msgs/msg/Path`，当前局部路径
- `/planning/markers`：`visualization_msgs/msg/MarkerArray`
- `/cmd_vel`：`geometry_msgs/msg/Twist`
- `/wheel_cmd`：`std_msgs/msg/Float64MultiArray`，`[left_rad_s, right_rad_s]`
- `/docking/status`：`std_msgs/msg/String`

## 编译与运行

```bash
cd /home/xmz/Dock_ws
colcon build --symlink-install --packages-select Docking_pkg
source install/setup.bash

# 仅规划节点，接收真实定位
ros2 launch Docking_pkg docking_planner.launch.py

# 闭环仿真：目标沿曲线运动，自车由 wheel_cmd 积分驱动
ros2 launch Docking_pkg docking_sim.launch.py

# 闭环仿真 + RViz 可视化
ros2 launch Docking_pkg docking_rviz.launch.py
```

`docking_rviz.launch.py` 会启动：

- `demo_sim_node`：模拟定位并发布车体位置；
- `docking_planner_node`：滚动规划与对接控制；
- `rviz2`：加载 `config/docking.rviz`。

RViz 中可以看到：

- 绿色方块：自车，车头有黄色锥形对接头；
- 红色方块：目标车，车尾有灰色凹槽；
- `/planning/path`：每次滚动生成的局部路径；
- `/planning/markers`：自车轨迹、目标 CTRV 预测轨迹、动态对接目标点；
- `map -> ego_base / target_base` TF 树。

## 二维码定位与 odom 坐标转换

二维码定位输出的是标签相对相机的位姿：

```text
T_camera_tag
```

适配节点按下面的链路转换到统一规划坐标系：

```text
T_odom_target_base =
    T_odom_ego
  * T_ego_camera
  * T_camera_tag
  * T_tag_target_base
```

仿真中发布的 TF 链为：

```text
odom -> ego_base -> camera_link -> qr_tag -> target_base
```

其中：

- `odom -> ego_base`：自车定位；
- `ego_base -> camera_link`：相机安装静态外参；
- `camera_link -> qr_tag`：二维码相对相机位姿；
- `qr_tag -> target_base`：二维码标签到目标车底盘的静态外参。

规划器只接收 `odom` 下的 `/target/pose`，内部不再直接处理相机坐标系。

## 速度规划与追赶逻辑

当目标在前方时，状态机用目标航向投影计算纵向间距：

```text
longitudinal_error = dot(target - ego, target_heading) - desired_center_distance
```

追赶速度按比例生成：

```text
closing_speed = clip(catch_up_gain * longitudinal_error, -max_closure, +max_closure)
ego_speed_cmd = clip(target_speed + closing_speed, 0, max_ego_speed)
```

因此目标越远，自车速度高于目标速度的追赶余量越大；接近到目标尾部时，追赶余量自动收敛，最后速度匹配目标速度，进入 `ALIGN / FINAL_DOCK / DOCKED`。相关参数在 `docking` 段里的 `catch_up_gain`、`max_closure_speed`、`min_closure_speed`。

## 提前对准

为了避免“快贴到目标才突然转向”，参考线生成时增加了预对准偏移：

- 追赶阶段，参考线终点会比正常对接点再后退 `prealign_distance`；
- 该偏移随目标与自车的间距缩小而平滑衰减；
- 控制器前视距离随速度变化，速度越高前视越远，从而更早开始转向；
- 接近后前视距离自动缩短，保证末端对接精度。

相关参数在 `planning` 和 `control` 段：

```yaml
planning:
  prealign_distance: 0.2
  prealign_activation_distance: 1.2
  docking_goal_lead: 0.03

control:
  lookahead_distance: 0.12
  lookahead_time: 0.30
  max_lookahead_distance: 0.30
  path_heading_gain: 2.5
  path_feedforward_gain: 0.8
```

## 目标预测稳定性

目标只有 XY 定位时，目标航向和角速度不是直接测量量。若用相邻两帧航向直接差分，定位噪声会被放大成很大的 `omega`，再经过 CTRV 积分就会在 RViz 中出现左右摆动甚至绕圈。

当前实现做了三层处理：

- `TargetStateEstimator` 用滑动时间窗对目标航向做最小二乘回归，避免用两帧差分估计 `omega`；
- 对回归结果做低通滤波、死区和小角速度置零，并限制最大合理角速度；
- `TargetPredictor` 对 CTRV 的 `omega` 再做限幅和死区处理，预测时域从原来的 10 s 缩短为 8 s。

这些参数都在 `config/docking_planner.yaml` 的 `target` 和 `prediction` 段中，可以根据实车定位噪声频率和平台实际最大转弯角速度调整。

查看状态：

```bash
ros2 topic echo /docking/status
ros2 topic echo /wheel_cmd
```

参数在 `config/docking_planner.yaml`。默认 `planner_mode: frenet`，可以改成 `cartesian` 对比两种局部规划路径。若定位系统使用自定义 XY 话题，可在适配节点中把 XY 转成 `PoseStamped` 后发布到 `/target/pose` 和 `/ego/pose`。
