#pragma once

#include "gps_transformer/TaskBase.hpp"

namespace gps_transformer {

using Pose = Eigen::Affine3d;
using BasePose = base::samples::RigidBodyState;

class Task : public TaskBase {
  public:
    explicit Task(std::string const& name = "gps_transformer::Task")
            : TaskBase(name),
              lastPose_(Pose::Identity()) {}

  protected:
    void inputPoseTransformerCallback(
            const base::Time& timestamp,
            const BasePose& gpsBasePose) override {
        if (!initialBasePose_.hasValidPosition())
            initialBasePose_ = gpsBasePose;

        BasePose diffPose = gpsBasePose;
        diffPose.position -= initialBasePose_.position;

        Pose yawRot, pose, deltaPose;
        yawRot = Eigen::AngleAxisd(_yawOffset.get(), Eigen::Vector3d::UnitZ());
        pose = yawRot * diffPose.getTransform();
        deltaPose = lastPose_.inverse() * pose;
        lastPose_ = pose;

        BasePose outputBaseDeltaPose, outputBasePose;
        outputBaseDeltaPose.setTransform(deltaPose);
        outputBasePose.setTransform(pose);

        _outputDeltaPose.write(outputBaseDeltaPose);
        _outputPose.write(outputBasePose);
    }

  protected:
    BasePose initialBasePose_;
    Pose lastPose_;
};

}  // namespace gps_transformer

