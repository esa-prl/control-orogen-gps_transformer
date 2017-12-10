#pragma once

#include "gps_transformer/TaskBase.hpp"

namespace gps_transformer {

using Pose = base::samples::RigidBodyState;

class Task : public TaskBase {
  public:
    explicit Task(std::string const& name = "gps_transformer::Task")
            : TaskBase(name) {}

  protected:
    void inputPoseTransformerCallback(
            const base::Time& timestamp,
            const Pose& inputPose) override {
        if (!initialPose_.hasValidPosition())
            initialPose_ = inputPose;

        Pose outputPose = inputPose;
        outputPose.position = inputPose.position - initialPose_.position;

        Eigen::Affine3d tf = outputPose.getTransform();
        tf.prerotate(Eigen::AngleAxisd(_yawOffset.rvalue(),
                Eigen::Vector3d::UnitZ()));
        outputPose.setTransform(tf);

        _outputPose.write(outputPose);
    }

  protected:
    Pose initialPose_;
};

}  // namespace gps_transformer

