# 可重构平台对接项目进度汇报

## 一、项目目标

两个可重构移动平台，目标平台在前方运动，自车平台从后方追赶，在无避障、仅依赖二维码定位的条件下完成对接。

系统需要满足：

- 自车不知道目标车辆的绝对速度和绝对方向；
- 自车没有全局定位，只能获得自车轮速和 IMU 横摆角；
- 二维码只提供标签相对相机的三维位置，不提供目标朝向；
- 自车需要通过相对运动估计、预测和滚动规划，最终与目标平台贴合。

---

## 二、当前传感器与输入条件

| 输入 | 类型 | 说明 |
| --- | --- | --- |
| 自车轮速 | 左右轮角速度 | 用于计算自车前进速度 |
| 自车横摆角 ψ | IMU 角度 | 用于差分得到横摆角速度 ω |
| 二维码位置 | (x_c, y_c, z_c) | 标签相对相机的三维位置，无朝向 |

当前没有：

- 自车全局 odom / map 坐标；
- 目标车辆速度传感器；
- 目标车辆朝向传感器；
- 全局定位模块。

---

## 三、总体技术架构

```
┌────────────────────────────────────────┐
│              传感器输入                 │
│ 车轮速度 v_e | IMU 横摆角 ψ_e           │
│ 二维码三维位置 (x_c, y_c, z_c)          │
└──────────────────┬─────────────────────┘
                   ▼
┌────────────────────────────────────────┐
│ 模块 1：坐标转换与数据同步              │
│ PositionProcessor                       │
│ 相机系 → 车体系，标签外参补偿，异常检查 │
└──────────────────┬─────────────────────┘
                   ▼
┌────────────────────────────────────────┐
│ 模块 2：相对状态估计                    │
│ EgoMotionCompensator + TargetVelocityFilter │
│ 自车运动补偿、坐标系旋转补偿、速度滤波  │
└──────────────────┬─────────────────────┘
                   ▼
┌────────────────────────────────────────┐
│ 模块 3：目标运动预测                    │
│ 局部坐标系下 CV 预测 predictRelative    │
└──────────────────┬─────────────────────┘
                   ▼
┌────────────────────────────────────────┐
│ 模块 4：参考轨迹生成                    │
│ 自车原点 → 目标后方对接点               │
└──────────────────┬─────────────────────┘
                   ▼
┌────────────────────────────────────────┐
│ 模块 5：速度规划与对接状态机            │
│ 对接时刻、末端闭合、DOCKED 判定         │
└──────────────────┬─────────────────────┘
                   ▼
┌────────────────────────────────────────┐
│ 模块 6：控制与滚动重规划                │
│ 差速轮速输出，QR 与控制频率解耦         │
└────────────────────────────────────────┘
```

当前第一版为了先跑通闭环，采用：

```
规划坐标系 = ego_base（自车车体系）
自车固定为 (0, 0)，朝向 0
目标为相对位置
目标只走直线，目标航向固定为 0
无 odom，无全局坐标
```

---

## 四、关键模块与代码位置

### 1. PositionProcessor

文件：

- `include/docking_pkg/relative_state_estimator.hpp`
- `src/relative_state_estimator.cpp`

职责：

1. 相机坐标 → 自车坐标：

```
p_ego = R(camera_yaw) * p_camera + camera_offset
```

2. 二维码标签安装偏移补偿：

```
target_center = tag_origin - R(tag_yaw) * tag_offset
```

3. 位置异常检查：

```
min_range < |p| < max_range
```

其中第一版使用固定标签安装角，因为二维码不提供目标朝向。

### 2. EgoMotionCompensator

职责：

- 读取自车轮速，得到前进速度 v_e；
- 读取 IMU 横摆角 ψ_e；
- 由横摆角差分得到横摆角速度：

```
raw_omega = normalizeAngle(psi_k - psi_{k-1}) / dt
omega = low_pass(raw_omega)
```

- 对相对运动做坐标系旋转补偿：

```
v_rel_x = dp_x/dt - omega_e * p_y
v_rel_y = dp_y/dt + omega_e * p_x
```

这一步解决了“不能直接把二维码位置差分当作目标真实速度”的问题。

### 3. TargetVelocityFilter

职责：

- 估计目标相对纵向速度 vx；
- 估计目标相对横向速度 vy；
- 低通滤波；
- 限制最大速度；
- 输出相对 `TargetState`。

第一版因为目标直线行驶，目标航向固定为：

```
target.yaw = 0
target.omega = 0
```

### 4. TargetPredictor

文件：

- `include/docking_pkg/target_predictor.hpp`
- `src/target_predictor.cpp`

车体系模式下使用相对速度直接积分：

```
x(t+dt) = x + vx * dt
y(t+dt) = y + vy * dt
```

对应函数：

```
TargetPredictor::predictRelative()
```

这样避免了 CTRV 模型沿目标航向积分时把相对速度符号丢掉的问题。

### 5. 参考轨迹与对接点

对接点定义：

```
goal_x = target.x - desired_center_distance * cos(target.yaw)
goal_y = target.y - desired_center_distance * sin(target.yaw)
```

其中：

```
desired_center_distance = 0.52 m
```

含义：

```
目标车后壁 与 自车前脸 相贴
```

参考轨迹：

```
起点：自车原点 (0, 0)
终点：目标后方对接点
形状：车体系下的稳定直线路径
```

### 6. 速度规划与状态机

状态机阶段：

```
WAITING_FOR_TARGET
TRACKING
APPROACH
ALIGN
FINAL_DOCK
DOCKED
```

末端行为：

- `final_closure_speed = 0.04 m/s`
- `docked_longitudinal_error = 0.02 m`
- `docked_release_margin = 0.30 m`
- `DOCKED` 为锁存状态，避免噪声导致状态反复跳变

### 7. 控制

文件：

- `include/docking_pkg/differential_controller.hpp`
- `src/differential_controller.cpp`

两种模式：

- 全局/odom 模式：路径跟踪器 `compute()`
- 车体系模式：相对位姿控制器 `computeBodyRelative()`

车体系控制律：

```
along      = 目标在目标航向上的投影
lateral    = 目标横向偏差
v_along    = 目标相对纵向速度
v_lateral  = 目标相对横向速度

linear  = ego_speed + k_along_vel * (v_along - desired_v_along)
angular = target.omega + k_heading * target.yaw
        - k_lateral * lateral - k_lateral_vel * v_lateral
```

最后转换为左右轮角速度：

```
v_l = linear - angular * wheel_track / 2
v_r = linear + angular * wheel_track / 2
wheel_left  = v_l / wheel_radius
wheel_right = v_r / wheel_radius
```

### 8. 仿真与可视化

仿真节点：

- `src/demo_sim_node.cpp`

发布：

- `/ego/pose`：自车位姿（仿真真值）
- `/ego/twist`：自车速度
- `/ego/yaw`：自车横摆角
- `/qr/tag_pose`：二维码相对相机位姿
- `/qr/tag_position`：二维码相对相机三维位置（规划器实际使用）
- `/simulation/markers`：车体、锥套、方向箭头
- TF：`odom -> ego_base`、`ego_base -> camera_link`、`camera_link -> qr_tag`、`qr_tag -> target_base`

RViz：

- `config/docking.rviz`
- 当前 Fixed Frame 为 `ego_base`
- 蓝色线为车体系下的相对参考路径

---

## 五、从初版到现在的关键改动回顾

1. 目标状态估计由简单差分改为卡尔曼滤波。
2. 目标预测升级为 CTRV，后因相对速度符号问题在车体系模式改为 CV 相对预测。
3. 参考线生成加入动态对接时间 td 和预对准偏移。
4. Frenet 局部规划修正 dl/ds、起点横向导数和偏移曲线曲率。
5. 速度规划补齐最大加速度、最大减速度、终点不越界和加速度输出。
6. 增加二维码适配节点，建立 `camera -> tag -> target_base` 坐标转换。
7. 发现并修正 tag→target 外参乘反的问题（曾导致目标位置偏一个车身长度）。
8. 增加 RViz 仿真、车体 Marker、锥套 Marker 和 TF。
9. 增加自车/目标速度话题与速度输入。
10. 改为车体系规划：无 odom，自车固定为原点，目标为相对状态。
11. 拆分相对状态估计为 PositionProcessor、EgoMotionCompensator、TargetVelocityFilter。
12. IMU 输入由横摆角速度改为横摆角，通过差分得到横摆角速度并滤波。
13. 增加目标位置异常检查、启动预热、DOCKED 锁存。
14. 车体系模式下使用相对闭合速度判断 DOCKED。

---

## 六、仿真验证结果

### 1. 远距离直线目标

目标初始位于自车前方约 10 m，直线行驶，速度 0.35 m/s。

结果：

```
TRACKING → APPROACH → ALIGN → FINAL_DOCK → DOCKED
最终中心距约 0.52 m
横向误差约 0
航向误差约 0
```

### 2. 短距离直线目标

目标初始位于自车前方 1.0 m，直线行驶。

结果：

```
约 4 ~ 6 秒进入 DOCKED
dist    ≈ 0.50 ~ 0.54 m
lat_err ≈ 0.00 m
yaw_err ≈ 0.00 rad
```

### 3. 最小对接距离测试

| 初始中心距 | 结果 |
| --- | --- |
| 1.0 m | 正常对接 |
| 0.7 m | 正常对接 |
| 0.55 m | 正常对接 |
| ≤ 0.54 m | 已处于贴合范围，直接进入 DOCKED |

结论：

```
几何最小中心距：0.52 m
当前代码实测最小初始中心距：约 0.55 m
```

影响最小距离的因素：

- 平台长度、圆锥突出、凹槽深度；
- 最大加减速度、最大速度、目标速度；
- 二维码频率、噪声、延迟；
- 控制周期与估计收敛时间；
- 初始横向与航向偏差；
- 机械自对准能力。

工程估算：

```
d0_min ≈ d_geo
       + v_rel * t_delay
       + v_rel^2 / (2 * a_decel)
       + d_align
       + margin
```

---

## 七、当前算法运行流程

```
1. 读取自车轮速和 IMU 横摆角
2. 读取二维码三维位置
3. PositionProcessor：
   相机系 → 车体系，标签偏移补偿，异常检查
4. EgoMotionCompensator：
   由横摆角差分估计 omega
   计算相对速度 v_rel = dp/dt + omega × p
5. TargetVelocityFilter：
   低通滤波得到 vx, vy
6. 目标预测：
   x(t+dt) = x + vx * dt
   y(t+dt) = y + vy * dt
7. 计算对接点：
   goal = target - desired_center_distance * direction
8. 生成参考路径：
   从自车原点到对接点
9. 速度与末端控制：
   相对距离 + 相对速度 → linear / angular
10. 差速运动学：
    linear / angular → 左右轮速
11. 滚动重规划：
    QR 新帧到来时更新状态
    每个控制周期重新预测、规划和输出
```

---

## 八、当前限制

1. 目标只支持直线行驶，目标航向固定为 0；
2. 二维码只给位置，无法直接观测目标朝向；
3. 相对速度由单步差分 + 运动补偿 + 低通滤波，噪声较大；
4. 没有全局 odom，无法输出世界坐标系轨迹；
5. 没有避障，没有障碍物代价地图；
6. 目标速度很高时，最小初始距离会显著增加；
7. 机械上 8 cm 圆锥与 5 cm 凹槽会导致端面无法完全贴合，需要硬件确认。

---

## 九、下一步计划

1. 目标航向估计
   - 由目标相对位置序列估计目标运动方向；
   - 加入自车旋转补偿；
   - 支持目标转弯场景。

2. 相对速度估计改进
   - 使用滑动窗口最小二乘替代单步差分；
   - 引入速度卡尔曼滤波；
   - 降低传感器噪声影响。

3. 时空轨迹规划
   - 把对接时间 td 与速度规划真正耦合；
   - 输出与目标未来状态对应的时空轨迹；
   - 对接阶段匹配目标相对速度。

4. 短距离模式
   - 自适应关闭预对准偏移；
   - 调整最小对接时间；
   - 降低末端速度，增加安全裕量。

5. 仿真与实车验证
   - 扫描不同初始距离、目标速度、噪声、延迟；
   - 确定不同工况下的最小初始距离；
   - 用实车标定相机外参、标签外参和机械对接深度。

---

## 十、汇报要点（可直接口头汇报）

1. 完成了“二维码相对定位 + 自车轮速/IMU + 目标预测 + 滚动规划 + 差速控制”的完整闭环。
2. 解决了二维码只给位置、自车无 odom、目标无速度传感器条件下的相对状态估计问题。
3. 实现了车体系规划：自车固定为原点，目标为相对运动，避免了对全局坐标的依赖。
4. 仿真验证了目标直线运动下的追赶与对接：最终中心距约 0.52 m，横向与航向误差接近 0。
5. 当前最小初始中心距实测约 0.55 m，几何最小值为 0.52 m。
6. 下一阶段重点是目标航向估计、相对速度滤波改进和时空轨迹规划。

---

## 十一、运行方式

```bash
cd /home/xmz/Dock_ws
colcon build --symlink-install --packages-select Docking_pkg
source install/setup.bash

# 无界面闭环仿真
ros2 launch Docking_pkg docking_sim.launch.py

# RViz 可视化仿真
ros2 launch Docking_pkg docking_rviz.launch.py
```
