#pragma once

#include "docking_pkg/types.hpp"

namespace docking
{

class LocalPlanner
{
public:
  enum class Mode
  {
    kCartesian,
    kFrenet,
  };

  struct Params
  {
    Mode mode{Mode::kCartesian};
    int sample_count{80};
  };

  LocalPlanner();
  explicit LocalPlanner(const Params & params);

  Path plan(const Pose2D & ego_pose, const Path & reference_line) const;

private:
  Path planCartesian(const Pose2D & ego_pose, const Path & reference_line) const;
  Path planFrenet(const Pose2D & ego_pose, const Path & reference_line) const;

  Params params_;
};

}  // 命名空间 docking
