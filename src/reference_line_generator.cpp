#include "docking_pkg/reference_line_generator.hpp"

#include <algorithm>
#include <cmath>

#include "docking_pkg/motion_utils.hpp"
#include "docking_pkg/quintic.hpp"

namespace docking
{

namespace
{

Pose2D dockingPoseFromPoint(
  const TrajectoryPoint & target,
  const ReferenceLineGenerator::Params & params)
{
  Pose2D goal;

  goal.yaw = target.yaw;

  const double geometric_center_distance =
    params.target_length / 2.0 +
    params.ego_length / 2.0 +
    2.0 * params.cone_bus_length -
    params.docking_depth;
  const double desired_center_distance =
    params.desired_center_distance > 0.0
    ? params.desired_center_distance
    : geometric_center_distance;
  const double behind = desired_center_distance - params.docking_goal_lead;

  goal.x =
    target.x -
    std::cos(target.yaw) * behind;

  goal.y =
    target.y -
    std::sin(target.yaw) * behind;

  return goal;
}

Path sampleReferencePath(
  const Pose2D & start,
  const DockingGoal & goal,
  const ReferenceLineGenerator::Params & params,
  int samples)
{
  samples = std::max(3, samples);

  const double chord =
    distance(
      start.x,
      start.y,
      goal.pose.x,
      goal.pose.y);

  // 五次多项式曲线端点切向量尺度。
  // 与 Ego → Docking Goal 的直线距离相关，
  // 避免短距离 docking 时出现明显 overshoot。
  const double scale =
    std::max(
      0.05,
      chord * params.derivative_scale);

  const double dx0 =
    std::cos(start.yaw) * scale;

  const double dy0 =
    std::sin(start.yaw) * scale;

  const double dx1 =
    std::cos(goal.pose.yaw) * scale;

  const double dy1 =
    std::sin(goal.pose.yaw) * scale;

  // 第一版暂时假设自车起点曲率为 0。
  // 后续如果 EgoState 提供 kappa，
  // 可以直接使用实际起点曲率。
  const double start_kappa = 0.0;

  // 对接目标点的终点曲率来自目标运动状态：
  //
  // kappa = omega / v
  const double end_kappa =
    goal.speed > 1e-3
    ? goal.omega / goal.speed
    : 0.0;

  const double ddx0 =
    -std::sin(start.yaw) *
    scale * scale *
    start_kappa;

  const double ddy0 =
    std::cos(start.yaw) *
    scale * scale *
    start_kappa;

  const double ddx1 =
    -std::sin(goal.pose.yaw) *
    scale * scale *
    end_kappa;

  const double ddy1 =
    std::cos(goal.pose.yaw) *
    scale * scale *
    end_kappa;

  const auto cx =
    fitQuintic(
      start.x,
      dx0,
      ddx0,
      goal.pose.x,
      dx1,
      ddx1);

  const auto cy =
    fitQuintic(
      start.y,
      dy0,
      ddy0,
      goal.pose.y,
      dy1,
      ddy1);

  Path path;

  path.reserve(
    static_cast<size_t>(samples + 1));

  double previous_x = start.x;
  double previous_y = start.y;
  double s = 0.0;

  for (int i = 0; i <= samples; ++i) {

    const double u =
      static_cast<double>(i) /
      static_cast<double>(samples);

    const double x =
      evalQuintic(cx, u);

    const double y =
      evalQuintic(cy, u);

    const double dxdu =
      evalQuintic(cx, u, 1);

    const double dydu =
      evalQuintic(cy, u, 1);

    const double ddxdu =
      evalQuintic(cx, u, 2);

    const double ddydu =
      evalQuintic(cy, u, 2);

    if (i > 0) {
      s += distance(
        previous_x,
        previous_y,
        x,
        y);
    }

    PathPoint point;

    point.x = x;
    point.y = y;

    point.yaw =
      std::atan2(
        dydu,
        dxdu);

    point.s = s;

    const double denominator =
      std::pow(
        dxdu * dxdu +
        dydu * dydu,
        1.5);

    point.kappa =
      denominator > 1e-9
      ? (dxdu * ddydu - dydu * ddxdu) /
        denominator
      : 0.0;

    path.push_back(point);

    previous_x = x;
    previous_y = y;
  }

  return path;
}

}  // 命名空间

ReferenceLineGenerator::ReferenceLineGenerator()
: ReferenceLineGenerator(Params{})
{
}

ReferenceLineGenerator::ReferenceLineGenerator(
  const Params & params)
: params_(params)
{
}

Path ReferenceLineGenerator::generateBodyRelativePath(
  const Pose2D & ego_pose,
  const DockingGoal & goal) const
{
  return sampleReferencePath(
    ego_pose,
    goal,
    params_,
    params_.sample_count);
}

ReferenceLine ReferenceLineGenerator::generate(
  const Pose2D & ego_pose,
  double ego_speed,
  const Trajectory & predicted_target) const
{
  ReferenceLine reference;

  if (predicted_target.empty()) {
    return reference;
  }

  // ============================================================
  // 1. 确定预测起始时间
  // ============================================================

  const double start_time =
    predicted_target.front().t;

  bool found = false;

  double docking_time = predicted_target.back().t;
  double docking_speed = predicted_target.back().v;
  double docking_omega = predicted_target.back().omega;

  // 默认值：预测轨迹的最后一个点
  TrajectoryPoint docking_target_point = predicted_target.back();

  // ============================================================
  // 2. 遍历目标未来轨迹
  //
  // 对每一个未来时刻：
  //
  // 目标状态 Target(t)
  //      ↓
  // 对接点 DockingPoint(t)
  //      ↓
  // 自车到对接点
  //      ↓
  // 最短到达时间
  //
  // 找到最早可达的 docking time。
  // ============================================================

  for (const auto & point : predicted_target) {

    const double relative_time =
      point.t - start_time;

    // 不允许过早 docking
    if (relative_time <
      params_.min_docking_time)
    {
      continue;
    }

    // 不允许无限等待
    if (relative_time >
      params_.max_docking_time)
    {
      break;
    }

    DockingGoal candidate;

    candidate.valid = true;

    // 绝对时间
    candidate.time = point.t;

    // 目标在当前未来时刻对应的 docking pose
    candidate.pose =
      dockingPoseFromPoint(
        point,
        params_);

    // Docking 时目标的速度和角速度
    candidate.speed = point.v;
    candidate.omega = point.omega;

    // ----------------------------------------------------------
    // 3. 低分辨率生成候选 Reference Path
    // ----------------------------------------------------------

    const int coarse_samples =
      std::max(
        10,
        params_.sample_count / 4);

    const Path candidate_path =
      sampleReferencePath(
        ego_pose,
        candidate,
        params_,
        coarse_samples);

    // ----------------------------------------------------------
    // 4. 使用实际参考路径长度估计 Ego 最短到达时间
    // ----------------------------------------------------------

    const double effective_distance =
      candidate_path.empty()
      ? distance(
          ego_pose.x,
          ego_pose.y,
          candidate.pose.x,
          candidate.pose.y)
      : candidate_path.back().s *
        params_.path_length_margin;

    const double required_time =
      minimumTravelTime(
        effective_distance,
        ego_speed,
        candidate.speed,
        params_.max_ego_accel,
        params_.max_ego_decel,
        params_.max_ego_speed);

    // ----------------------------------------------------------
    // 5. 判断 Ego 是否能在该 docking time 到达
    // ----------------------------------------------------------

  if (std::isfinite(required_time) && required_time <= relative_time + 1e-6)
  {
    docking_time = candidate.time;
    docking_speed = candidate.speed;
    docking_omega = candidate.omega;

    // 保存选中时刻的目标预测状态
    docking_target_point = point;

    found = true;
    break;
  }
  }

  // ============================================================
  // 6. 没有找到可行 Docking Time
  // ============================================================

  if (!found) {
    docking_time = predicted_target.back().t;
    docking_speed = predicted_target.back().v;
    docking_omega = predicted_target.back().omega;
  }

  // ============================================================
  // 7. 生成最终高分辨率 Reference Line
  // ============================================================

  // 路径终点连接到当前目标对接点。
  // 选中的 docking_time 仅用于速度规划，避免
  // 远未来交会点随着
  // 自车接近而向自车方向后退。
  DockingGoal path_goal;
  path_goal.valid = found;
  path_goal.time = docking_time;
  path_goal.speed = docking_speed;
  path_goal.omega = docking_omega;
  // 如果找到了可行的对接时刻，使用对应的未来目标状态。
// 如果没有找到，则暂时退回当前目标状态，保持滚动追踪。
  const TrajectoryPoint & path_target_point = found ? docking_target_point : predicted_target.front();

  path_goal.pose = dockingPoseFromPoint( path_target_point,params_);

  const double current_gap =
    distance(ego_pose.x, ego_pose.y, predicted_target.front().x,
      predicted_target.front().y) - desired_center_distance();
  const double blend = clip(
    current_gap / std::max(0.1, params_.prealign_activation_distance),
    0.0, 1.0);
  const double smooth_blend = blend * blend * (3.0 - 2.0 * blend);
  const double extra_offset = params_.prealign_distance * smooth_blend;
  path_goal.pose.x -= std::cos(path_goal.pose.yaw) * extra_offset;
  path_goal.pose.y -= std::sin(path_goal.pose.yaw) * extra_offset;

  reference.path = sampleReferencePath(
    ego_pose,
    path_goal,
    params_,
    params_.sample_count);
  reference.docking_goal = path_goal;

  return reference;
}

}  // 命名空间 docking
