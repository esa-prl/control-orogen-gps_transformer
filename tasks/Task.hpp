#pragma once

#include "gps_transformer/TaskBase.hpp"

namespace gps_transformer {

using Pose = base::samples::RigidBodyState;

class Task : public TaskBase {
  friend class TaskBase;

  public:
    explicit Task(std::string const& name = "gps_transformer::Task")
            : TaskBase(name) {}

  protected:
    void inputPoseCallback(
            const base::Time& timestamp,
            const Pose& inputPose) override {}
};

}  // namespace gps_transformer

