## 1. 统一的规划坐标系

规划坐标系是：

```text
odom
```

这是一个**固定世界坐标系**，不是以自车或目标车为中心的坐标系。

- 自车位姿、目标车位姿、目标预测轨迹、参考线、局部路径、速度规划都在 `odom` 下；
- `nav_msgs/Path` 和 RViz Marker 的 `frame_id` 都是 `odom`；
- RViz 的 Fixed Frame 也是 `odom`。

## 2. 当前坐标树

```text
odom
 └── ego_base
      └── camera_link
           └── qr_tag
                └── target_base
```

各坐标系含义：

| 坐标系 | 含义 |
| --- | --- |
| `odom` | 固定规划坐标系 |
| `ego_base` | 自车底盘坐标系 |
| `camera_link` | 自车相机坐标系 |
| `qr_tag` | 二维码标签坐标系 |
| `target_base` | 目标车底盘坐标系 |

其中：

- `odom -> ego_base`：自车定位给出，动态变化；
- `ego_base -> camera_link`：相机安装外参，静态；
- `camera_link -> qr_tag`：二维码相对相机的位姿，动态变化；
- `qr_tag -> target_base`：标签相对目标车底盘的外参，静态。

## 3. 二维码坐标转换流程

二维码定位直接给出的是：
```text
T_camera_tag
```
也就是二维码标签相对相机的位姿。

### 第一步：转到自车相机

自车定位给出：

```text
T_odom_ego
```

相机安装外参给出：

```text
T_ego_camera
```

得到相机在 `odom` 下的位姿：

```text
T_odom_camera = T_odom_ego * T_ego_camera
```

### 第二步：把二维码转到 odom

```text
T_odom_tag = T_odom_camera * T_camera_tag
```

### 第三步：转到目标车底盘

标签安装外参给出：

```text
T_target_tag
```

含义是：tag 在目标车底盘坐标系下的位姿。

因此目标车底盘在 `odom` 下的位姿是：

```text
T_odom_target = T_odom_tag * T_target_tag^{-1}
```

类型是 `geometry_msgs/msg/PoseStamped`，`header.frame_id = odom`，内容是目标车底盘在 `odom` 下的位姿。

## 4. 规划器实际使用的输入

规划节点 [docking_planner_node.cpp](/home/xmz/Dock_ws/src/Docking_pkg/src/docking_planner_node.cpp) 只接收已经统一到 `odom` 的数据：

| 话题 | 坐标系 | 含义 |
| --- | --- | --- |
| `/ego/pose` | `odom` | 自车位姿 |
| `/target/pose` | `odom` | 目标车位姿，由二维码适配节点输出 |
| `/ego/twist` | `odom` | 自车速度 |
| `/target/twist` | `odom` | 目标速度，可选 |

默认情况下，二维码适配节点不发布 `/target/twist`，因为二维码只给位姿时直接差分噪声太大；规划器用状态估计器从 `odom` 下的目标位姿估计目标速度。

如果以后二维码驱动能直接给目标速度，也可以打开适配节点的：

```yaml
publish_twist: true
```

但必须保证速度也在 `odom` 下。

## 5. 相对位置是怎么算的

两车位姿都已经在 `odom` 下，所以在 [docking_state_machine.cpp](/home/xmz/Dock_ws/src/Docking_pkg/src/docking_state_machine.cpp) 里直接相减：

```cpp
dx = target.x - ego.x;
dy = target.y - ego.y;
center_distance = hypot(dx, dy);
```

然后把相对位置投影到目标车航向上：

```text
longitudinal_error = dot(r_target_ego, target_heading) - desired_center_distance
lateral_error      = cross(target_heading, r_target_ego)
heading_error      = target_yaw - ego_yaw
```

这样就得到了：

- 两车中心距；
- 纵向误差；
- 横向误差；
- 航向误差。

这些量全部基于 `odom` 下的统一位姿计算。

## 6. 规划链路使用的坐标系

| 模块 | 使用的坐标系 |
| --- | --- |
| 目标状态估计 | `odom` |
| CTRV 目标预测 | `odom` |
| 动态参考线生成 | `odom` |
| Frenet/笛卡尔局部规划 | `odom` |
| 速度规划 | 沿 `odom` 下的路径弧长 |
| 对接状态机 | `odom` 下的相对位置 |
| 差速控制输入 | 自车车体坐标系 |
| `/cmd_vel` | 自车车体坐标系 |
| `/wheel_cmd` | 左右轮角速度 |

也就是说：

- **感知层**：二维码给的是 `camera -> tag`；
- **转换层**：统一转成 `odom -> target_base`；
- **规划层**：全部在 `odom` 下进行；
- **控制层**：再转成自车本体的 `v, omega` 和左右轮速。

## 7. 为什么用 odom 而不是以某辆车为中心

- 如果以目标车为中心，目标车一边走一边转，参考线和历史轨迹都要不断重算；
- 如果以自车为中心，自车一动，所有世界轨迹都要重新表达；
- 用 `odom` 后，两车、预测轨迹、参考线、局部路径都在同一个固定坐标系里，滚动规划才一致。

如果你以后有全局 SLAM，也可以把 `planning_frame` 改成 `map`：

```yaml
planning_frame: "map"
```

只要保证：

```text
map -> ego_base
map -> camera_link
qr_tag -> target_base
```

这些 TF 都存在，二维码适配节点输出的 `/target/pose` 就会自动落在 `map` 下。

总结一句话：

```text
二维码给出 camera -> tag；
适配节点转换为 odom -> target_base；
规划器全部在 odom 下工作；
控制量最后转回自车车体坐标。
```